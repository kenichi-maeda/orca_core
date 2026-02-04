#include "dynamixel_native.h"

#include <iostream>
#include <cstdint>
#include <cstring>
#include <thread>
#include <chrono>

using namespace dynamixel;

#define PROTOCOL_VERSION 2.0

// The following addresses assume XH motors.
// see https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/ for control table
#define ADDR_TORQUE_ENABLE        64
#define ADDR_OPERATING_MODE       11
#define ADDR_GOAL_POSITION        116
#define ADDR_GOAL_CURRENT         102
#define ADDR_PROFILE_VELOCITY     112
#define ADDR_PRESENT_CURRENT      126
#define ADDR_PRESENT_VELOCITY     128
#define ADDR_PRESENT_POSITION     132
#define ADDR_PRESENT_POS_VEL_CUR  126
#define LEN_PRESENT_POS_VEL_CUR   10
#define ADDR_MOVING_STATUS        123
#define ADDR_PRESENT_TEMPERATURE  146

static inline int32_t unsigned_to_signed(uint32_t value, int size_bytes)
{
    const int bits = size_bytes * 8;
    const uint32_t sign_bit = 1u << (bits - 1);
    if (value & sign_bit)
        return static_cast<int32_t>(value - (1u << bits));
    return static_cast<int32_t>(value);
}

static inline uint32_t signed_to_unsigned(int32_t value, int size_bytes)
{
    const int bits = size_bytes * 8;
    if (value < 0)
        return static_cast<uint32_t>((1u << bits) + value);
    return static_cast<uint32_t>(value);
}

DynamixelNative::DynamixelNative(const std::string& port_name,
                                 int baud,
                                 const std::vector<int>& ids)
    : motor_ids(ids), baudrate_(baud)
{
    port = PortHandler::getPortHandler(port_name.c_str());
    packet = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
}

void DynamixelNative::connect()
{
    if (!port->openPort())
        throw std::runtime_error("Failed to open Dynamixel port");

    if (!port->setBaudRate(baudrate_))
        throw std::runtime_error("Failed to set Dynamixel baudrate");
}

void DynamixelNative::disconnect()
{
    port->closePort();
}

void DynamixelNative::enable_torque(const std::vector<int>& ids,
                                    bool enable,
                                    int retries,
                                    double retry_interval_sec)
{
    while (true)
    {
        bool all_ok = true;
        std::vector<int> errored_ids;

        for (int id : ids)
        {
            uint8_t dxl_error = 0;
            int result = packet->write1ByteTxRx(
                port, id, ADDR_TORQUE_ENABLE,
                static_cast<uint8_t>(enable),
                &dxl_error);

            if (result != COMM_SUCCESS)
            {
                all_ok = false;
                errored_ids.push_back(id);
                std::cerr << "[ID " << id << "] "
                          << packet->getTxRxResult(result) << std::endl;
                continue;
            }
            if (dxl_error != 0)
            {
                all_ok = false;
                errored_ids.push_back(id);
                std::cerr << "[ID " << id << "] "
                          << packet->getRxPacketError(dxl_error) << std::endl;
            }
        }

        if (all_ok || retries == 0)
            return;

        if (!errored_ids.empty())
        {
            std::cerr << "enable_torque failed for IDs: ";
            for (size_t i = 0; i < errored_ids.size(); ++i)
            {
                std::cerr << errored_ids[i];
                if (i + 1 < errored_ids.size())
                    std::cerr << ", ";
            }
            std::cerr << std::endl;
        }

        if (retries > 0)
            --retries;

        std::this_thread::sleep_for(
            std::chrono::duration<double>(retry_interval_sec));
    }
}


void DynamixelNative::set_operating_mode(const std::vector<int>& ids, int mode)
{
    enable_torque(ids, false);

    std::vector<int> errored_ids;
    for (int id : ids)
    {
        uint8_t dxl_error = 0;
        int result = packet->write1ByteTxRx(
            port, id, ADDR_OPERATING_MODE,
            static_cast<uint8_t>(mode),
            &dxl_error);
        if (result != COMM_SUCCESS)
        {
            errored_ids.push_back(id);
            std::cerr << "[ID " << id << "] "
                      << packet->getTxRxResult(result) << std::endl;
            continue;
        }
        if (dxl_error != 0)
        {
            errored_ids.push_back(id);
            std::cerr << "[ID " << id << "] "
                      << packet->getRxPacketError(dxl_error) << std::endl;
        }
    }

    if (!errored_ids.empty())
    {
        std::cerr << "set_operating_mode failed for IDs: ";
        for (size_t i = 0; i < errored_ids.size(); ++i)
        {
            std::cerr << errored_ids[i];
            if (i + 1 < errored_ids.size())
                std::cerr << ", ";
        }
        std::cerr << std::endl;
    }

    enable_torque(ids, true);
}


void DynamixelNative::set_goal_position(const std::vector<int>& ids,
                                        const std::vector<int>& ticks)
{
    sync_write(ids, ticks, ADDR_GOAL_POSITION, 4);
}

void DynamixelNative::set_goal_current(const std::vector<int>& ids,
                                       const std::vector<int>& ticks)
{
    sync_write(ids, ticks, ADDR_GOAL_CURRENT, 2);
}

void DynamixelNative::set_profile_velocity(const std::vector<int>& ids,
                                           const std::vector<int>& ticks)
{
    sync_write(ids, ticks, ADDR_PROFILE_VELOCITY, 4);
}


void DynamixelNative::init_bulk_readers()
{
    if (!sync_pos_vel_cur)
    {
        sync_pos_vel_cur = std::make_unique<GroupSyncRead>(port, packet, ADDR_PRESENT_POS_VEL_CUR, LEN_PRESENT_POS_VEL_CUR);
        for (int id : motor_ids)
        {
            if (!sync_pos_vel_cur->addParam(id))
            {
                std::cerr << "[ID " << id << "] sync_pos_vel_cur addParam failed" << std::endl;
            }
        }
    }

    if (!bulk_temp)
    {
        bulk_temp = std::make_unique<GroupBulkRead>(port, packet);
        for (int id : motor_ids)
        {
            if (!bulk_temp->addParam(id, ADDR_PRESENT_TEMPERATURE, 1))
                std::cerr << "[ID " << id << "] bulk_temp addParam failed" << std::endl;
        }
    }

    if (!bulk_moving)
    {
        bulk_moving = std::make_unique<GroupBulkRead>(port, packet);
        for (int id : motor_ids)
        {
            if (!bulk_moving->addParam(id, ADDR_MOVING_STATUS, 1))
                std::cerr << "[ID " << id << "] bulk_moving addParam failed" << std::endl;
        }
    }
}


std::tuple<std::vector<float>, std::vector<float>, std::vector<float>>
DynamixelNative::read_pos_vel_cur()
{
    init_bulk_readers();
    int result = sync_pos_vel_cur->txRxPacket();
    if (result != COMM_SUCCESS)
        std::cerr << packet->getTxRxResult(result) << std::endl;


    std::vector<float> pos, vel, cur;
    pos.reserve(motor_ids.size());
    vel.reserve(motor_ids.size());
    cur.reserve(motor_ids.size());

    for (int id : motor_ids)
    {
        bool ok = sync_pos_vel_cur->isAvailable(id, ADDR_PRESENT_POS_VEL_CUR, LEN_PRESENT_POS_VEL_CUR);
        if (!ok)
        {
            std::cerr << "[ID " << id << "] sync_pos_vel_cur data unavailable" << std::endl;
            pos.push_back(0.0f);
            vel.push_back(0.0f);
            cur.push_back(0.0f);
            continue;
        }

        pos.push_back(unsigned_to_signed(
            sync_pos_vel_cur->getData(id, ADDR_PRESENT_POSITION, 4), 4));
        vel.push_back(unsigned_to_signed(
            sync_pos_vel_cur->getData(id, ADDR_PRESENT_VELOCITY, 4), 4));
        cur.push_back(unsigned_to_signed(
            sync_pos_vel_cur->getData(id, ADDR_PRESENT_CURRENT, 2), 2));
    }

    return {pos, vel, cur};
}

std::vector<float> DynamixelNative::read_temperature()
{
    init_bulk_readers();
    int result = bulk_temp->txRxPacket();
    if (result != COMM_SUCCESS)
        std::cerr << packet->getTxRxResult(result) << std::endl;

    std::vector<float> temps;
    temps.reserve(motor_ids.size());

    for (int id : motor_ids)
    {
        if (!bulk_temp->isAvailable(id, ADDR_PRESENT_TEMPERATURE, 1))
        {
            std::cerr << "[ID " << id << "] bulk_temp data unavailable" << std::endl;
            temps.push_back(0.0f);
            continue;
        }
        temps.push_back(static_cast<float>(
            bulk_temp->getData(id, ADDR_PRESENT_TEMPERATURE, 1)));
    }

    return temps;
}

std::vector<uint8_t> DynamixelNative::read_moving_status()
{
    init_bulk_readers();
    int result = bulk_moving->txRxPacket();
    if (result != COMM_SUCCESS)
        std::cerr << packet->getTxRxResult(result) << std::endl;

    std::vector<uint8_t> status;
    status.reserve(motor_ids.size());

    for (int id : motor_ids)
    {
        if (!bulk_moving->isAvailable(id, ADDR_MOVING_STATUS, 1))
        {
            std::cerr << "[ID " << id << "] bulk_moving data unavailable" << std::endl;
            status.push_back(0);
            continue;
        }
        status.push_back(
            bulk_moving->getData(id, ADDR_MOVING_STATUS, 1));
    }

    return status;
}

void DynamixelNative::sync_write(const std::vector<int>& ids,
                                 const std::vector<int>& values,
                                 int address,
                                 int size)
{
    if (ids.size() != values.size())
        throw std::runtime_error("sync_write: ids and values size mismatch");

    auto key = std::make_pair(address, size);

    if (!sync_writers.count(key))
    {
        sync_writers[key] =
            std::make_unique<GroupSyncWrite>(port, packet, address, size);
    }

    auto& writer = sync_writers[key];
    writer->clearParam();

    std::vector<int> errored_ids;
    for (size_t i = 0; i < ids.size(); ++i)
    {
        uint32_t val = signed_to_unsigned(values[i], size);
        uint8_t data[4] = {};
        std::memcpy(data, &val, size);
        if (!writer->addParam(ids[i], data))
            errored_ids.push_back(ids[i]);
    }

    if (!errored_ids.empty())
    {
        std::cerr << "sync_write addParam failed for IDs: ";
        for (size_t i = 0; i < errored_ids.size(); ++i)
        {
            std::cerr << errored_ids[i];
            if (i + 1 < errored_ids.size())
                std::cerr << ", ";
        }
        std::cerr << std::endl;
    }

    int result = writer->txPacket();
    if (result != COMM_SUCCESS)
        std::cerr << packet->getTxRxResult(result) << std::endl;
}
