#include "protocol/crc.h"

namespace gripper::protocol
{
uint16_t crc16Modbus(const uint8_t* data, std::size_t length)
{
    uint16_t crc = 0xFFFF;

    for (std::size_t i = 0; i < length; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]);

        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x0001)
            {
                crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

uint16_t crc16Modbus(const std::vector<uint8_t>& data)
{
    return crc16Modbus(data.data(), data.size());
}
}