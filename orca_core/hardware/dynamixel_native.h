#pragma once

#include <vector>
#include <tuple>
#include <string>
#include <stdexcept>
#include <memory>
#include <map>
#include <utility>


#include "dynamixel_sdk/dynamixel_sdk.h"

class DynamixelNative {
public:
    DynamixelNative(const std::string& port,
                    int baud,
                    const std::vector<int>& ids);

    void connect();
    void disconnect();

    void enable_torque(const std::vector<int>& ids,
                   bool enable,
                   int retries = -1,
                   double retry_interval_sec = 0.25);

    void set_operating_mode(const std::vector<int>& ids, int mode);

    // writes
    void set_goal_position(const std::vector<int>& ids, const std::vector<int>& ticks);
    void set_goal_current(const std::vector<int>& ids, const std::vector<int>& ticks);
    void set_profile_velocity(const std::vector<int>& ids, const std::vector<int>& ticks);
    void sync_write(const std::vector<int>& ids,
                    const std::vector<int>& values,
                    int address,
                    int size);

    // reads (pos, vel, cur)
    std::tuple<
        std::vector<float>,
        std::vector<float>,
        std::vector<float>
    > read_pos_vel_cur();

    std::vector<float> read_temperature();
    
    std::vector<uint8_t> read_moving_status();

    std::map<
    std::pair<int,int>,
    std::unique_ptr<dynamixel::GroupSyncWrite>
        > sync_writers;


    std::unique_ptr<dynamixel::GroupBulkRead> bulk_pos_vel_cur;
    std::unique_ptr<dynamixel::GroupBulkRead> bulk_temp;
    std::unique_ptr<dynamixel::GroupBulkRead> bulk_moving;


private:
    dynamixel::PortHandler* port;
    dynamixel::PacketHandler* packet;

    std::vector<int> motor_ids;
    int baudrate_;
    void init_bulk_readers();
};
