#ifndef GRIPPER_PROTOCOL_CRC_H
#define GRIPPER_PROTOCOL_CRC_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace gripper::protocol
{
uint16_t crc16Modbus(const uint8_t* data, std::size_t length);
uint16_t crc16Modbus(const std::vector<uint8_t>& data);
}

#endif