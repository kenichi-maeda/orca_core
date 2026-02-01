# Copyright 2019 The ROBEL Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Communication using the DynamixelSDK."""

import atexit
import logging
import time
import dynamixel_native
from typing import Optional, Sequence, Union, Tuple
import numpy as np

PROTOCOL_VERSION = 2.0

# The following addresses assume XH motors.
# see https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/ for control table
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_GOAL_PWM = 100
ADDR_GOAL_CURRENT = 102
ADDR_PROFILE_VELOCITY = 112
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POS_VEL_CUR = 126
ADDR_MOVING_STATUS = 123
ADDR_PRESENT_TEMPERATURE = 146

# Data Byte Length
LEN_OPERATING_MODE = 1
LEN_PRESENT_POSITION = 4
LEN_PRESENT_VELOCITY = 4
LEN_PRESENT_CURRENT = 2
LEN_PRESENT_POS_VEL_CUR = 10
LEN_GOAL_POSITION = 4
LEN_GOAL_PWM = 2
LEN_GOAL_CURRENT = 2
LEN_PROFILE_VELOCITY = 4
LEN_MOVING_STATUS = 1
LEN_PRESENT_TEMPERATURE = 1

DEFAULT_POS_SCALE = 2.0 * np.pi / 4096  # 0.088 degrees
# See http://emanual.robotis.com/docs/en/dxl/x/xh430-v210/#goal-velocity
DEFAULT_VEL_SCALE = 0.229 * 2.0 * np.pi / 60.0  # 0.229 rpm
DEFAULT_CUR_SCALE = 1.34



def dynamixel_cleanup_handler():
    """Ensure all clients disconnect cleanly on exit."""
    for client in list(DynamixelClient.OPEN_CLIENTS):
        try:
            client.disconnect()
        except Exception as e:
            logging.warning("Cleanup disconnect failed: %s", e)


class DynamixelClient:
    """Client for communicating with Dynamixel motors.

    NOTE: This only supports Protocol 2.
    """

    # The currently open clients.
    OPEN_CLIENTS = set()

    def __init__(self,
                 motor_ids: Sequence[int],
                 port: str = '/dev/ttyUSB0',
                 baudrate: int = 1000000,
                 lazy_connect: bool = False,
                 pos_scale: Optional[float] = None,
                 vel_scale: Optional[float] = None,
                 cur_scale: Optional[float] = None):
        """Initializes a new client.

        Args:
            motor_ids: All motor IDs being used by the client.
            port: The Dynamixel device to talk to. e.g.
                - Linux: /dev/ttyUSB0
                - Mac: /dev/tty.usbserial-*
                - Windows: COM1
            baudrate: The Dynamixel baudrate to communicate with.
            lazy_connect: If True, automatically connects when calling a method
                that requires a connection, if not already connected.
            pos_scale: The scaling factor for the positions. This is
                motor-dependent. If not provided, uses the default scale.
            vel_scale: The scaling factor for the velocities. This is
                motor-dependent. If not provided uses the default scale.
            cur_scale: The scaling factor for the currents. This is
                motor-dependent. If not provided uses the default scale.
        """
        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = baudrate
        self.lazy_connect = lazy_connect

        # Scaling factors: same semantics as original client.
        self.pos_scale = pos_scale or DEFAULT_POS_SCALE
        self.vel_scale = vel_scale or DEFAULT_VEL_SCALE
        self.cur_scale = cur_scale or DEFAULT_CUR_SCALE

        # Native C++ backend instance.
        self.native = dynamixel_native.Client(
            port=self.port_name,
            baudrate=self.baudrate,
            ids=self.motor_ids,
        )

        self._is_connected = False

        self.OPEN_CLIENTS.add(self)

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def _ensure_connected(self):
        if self.lazy_connect and not self.is_connected:
            self.connect()
        if not self.is_connected:
            raise OSError('Must call connect() first.')

    def connect(self):
        """Connects to the Dynamixel motors.

        NOTE: This should be called after all DynamixelClients on the same
            process are created.
        """
        assert not self.is_connected, 'Client is already connected.'

        # Open port & set baudrate inside C++.
        self.native.connect()

        # Start with all motors enabled, like original.
        self.native.enable_torque(self.motor_ids, True)

        self._is_connected = True
        logging.info('Connected to Dynamixel on port %s at %d baud.',
                     self.port_name, self.baudrate)

    def disconnect(self):
        """Disconnects from the Dynamixel device."""
        if not self.is_connected:
            return

        try:
            self.native.enable_torque(self.motor_ids, False)
        finally:
            self.native.disconnect()
            self._is_connected = False
            self.OPEN_CLIENTS.discard(self)

        logging.info("Disconnected from Dynamixel on %s", self.port_name)

    def set_torque_enabled(self,
                           motor_ids: Sequence[int],
                           enabled: bool,
                           retries: int = -1,
                           retry_interval: float = 0.25):
        """Sets whether torque is enabled for the motors.

        Args:
            motor_ids: The motor IDs to configure.
            enabled: Whether to engage or disengage the motors.
            retries: The number of times to retry. If this is <0, will retry
                forever.
            retry_interval: The number of seconds to wait between retries.
        """
        self._ensure_connected()
        self.native.enable_torque(
            list(motor_ids),
            bool(enabled),
            retries,
            retry_interval
        )

    def set_operating_mode(self, motor_ids: Sequence[int], mode_value: int):
        """
        see https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#operating-mode11
        0: current control mode
        1: velocity control mode
        3: position control mode
        4: multi-turn position control mode
        5: current-based position control mode
        """
        # data in EEPROM area can only be written when torque is disabled
        self._ensure_connected()
        self.native.set_operating_mode(list(motor_ids), int(mode_value))

    def read_pos_vel_cur(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns the positions, velocities, and currents (using native backend)."""
        
        self._ensure_connected()
        pos, vel, cur = self.native.read_pos_vel_cur()

        pos = np.asarray(pos, dtype=np.float32) * self.pos_scale
        vel = np.asarray(vel, dtype=np.float32) * self.vel_scale
        cur = np.asarray(cur, dtype=np.float32) * self.cur_scale

        return pos, vel, cur

    def read_status_is_done_moving(self) -> np.ndarray:
        """Returns boolean moving status for each motor."""
        self._ensure_connected()
        status = np.asarray(self.native.read_moving_status(), dtype=np.uint8)
        return (status & 0x01).astype(np.int8)

    def read_temperature(self) -> np.ndarray:
        """Returns motor temperatures in Celsius."""
        self._ensure_connected()
        return np.asarray(self.native.read_temperature(), dtype=np.float32)

    def write_desired_pos(self, motor_ids: Sequence[int],
                          positions: np.ndarray):
        """Writes the given desired positions.

        Args:
            motor_ids: The motor IDs to write to.
            positions: The joint angles in radians to write.
        """
        self._ensure_connected()

        motor_ids = list(motor_ids)
        positions = np.asarray(positions, dtype=np.float32)
        assert len(motor_ids) == len(positions)

        ticks = (positions / self.pos_scale).astype(np.int32)
        self.native.set_goal_position(motor_ids, ticks.tolist())

    def write_desired_current(self, motor_ids: Sequence[int], current: np.ndarray):
        self._ensure_connected()
        motor_ids = list(motor_ids)
        current = np.asarray(current, dtype=np.int32)
        assert len(motor_ids) == len(current)
        self.native.set_goal_current(motor_ids, current.tolist())

    def write_profile_velocity(self, motor_ids: Sequence[int], profile_velocity: np.ndarray):
        self._ensure_connected()
        motor_ids = list(motor_ids)
        profile_velocity = np.asarray(profile_velocity, dtype=np.int32)
        assert len(motor_ids) == len(profile_velocity)
        self.native.set_profile_velocity(motor_ids, profile_velocity.tolist())

    def sync_write(self,
                   motor_ids: Sequence[int],
                   values: np.ndarray,
                   address: int,
                   size: int):
        self._ensure_connected()

        motor_ids = list(motor_ids)
        values = np.asarray(values, dtype=np.int32)
        assert len(motor_ids) == len(values)

        self.native.sync_write(
            motor_ids,
            values.tolist(),
            int(address),
            int(size),
    )

    def __enter__(self):
        """Enables use as a context manager."""
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, *args):
        """Enables use as a context manager."""
        self.disconnect()

    def __del__(self):
        """Automatically disconnect on destruction."""
        self.disconnect()

# Register global cleanup function.
atexit.register(dynamixel_cleanup_handler)

if __name__ == '__main__':
    import argparse
    import itertools

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-m',
        '--motors',
        required=True,
        help='Comma-separated list of motor IDs.')
    parser.add_argument(
        '-d',
        '--device',
        default='/dev/cu.usbserial-FT62AFSR',
        help='The Dynamixel device to connect to.')
    parser.add_argument(
        '-b', '--baud', default=1000000, help='The baudrate to connect with.')
    parsed_args = parser.parse_args()
    motors = [int(motor) for motor in parsed_args.motors.split(',')]
    
    way_points = [np.zeros(len(motors)), np.full(len(motors), np.pi)]

    with DynamixelClient(motors, parsed_args.device,
                         parsed_args.baud) as dxl_client:
        for step in itertools.count():
            if step > 0 and step % 50 == 0:
                way_point = way_points[(step // 100) % len(way_points)]
                print('Writing: {}'.format(way_point.tolist()))
                dxl_client.write_desired_pos(motors, way_point)
            read_start = time.time()
            pos_now, vel_now, cur_now = dxl_client.read_pos_vel_cur()
            if step % 5 == 0:
                print('[{}] Frequency: {:.2f} Hz'.format(
                    step, 1.0 / (time.time() - read_start)))
                print('> Pos: {}'.format(pos_now.tolist()))
                print('> Vel: {}'.format(vel_now.tolist()))
                print('> Cur: {}'.format(cur_now.tolist()))
