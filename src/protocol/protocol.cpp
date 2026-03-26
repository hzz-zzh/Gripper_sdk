#include "protocol/protocol.h"
#include "protocol/crc.h"
#include <cstring>

namespace gripper::protocol
{
std::vector<uint8_t> buildFrame(uint8_t header,
                                uint8_t sequence,
                                uint8_t device,
                                uint8_t command,
                                const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> frame;
    frame.reserve(5 + payload.size() + 2);

    frame.push_back(header);
    frame.push_back(sequence);
    frame.push_back(device);
    frame.push_back(command);
    frame.push_back(static_cast<uint8_t>(payload.size()));
    frame.insert(frame.end(), payload.begin(), payload.end());

    const uint16_t crc = crc16Modbus(frame);
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

    return frame;
}

std::vector<uint8_t> buildRequest(uint8_t sequence,
                                  uint8_t device,
                                  Command command,
                                  const std::vector<uint8_t>& payload)
{
    return buildFrame(kMasterHeader,
                      sequence,
                      device,
                      static_cast<uint8_t>(command),
                      payload);
}

bool parseFrame(const std::vector<uint8_t>& raw, Frame& out, std::string* error)
{
    if (raw.size() < 7)
    {
        if (error) *error = "frame too short";
        return false;
    }

    const uint8_t length = raw[4];
    const std::size_t expected_size = 5 + static_cast<std::size_t>(length) + 2;

    if (raw.size() != expected_size)
    {
        if (error) *error = "frame size mismatch";
        return false;
    }

    const uint16_t crc_calc = crc16Modbus(raw.data(), raw.size() - 2);
    const uint16_t crc_recv = static_cast<uint16_t>(raw[raw.size() - 2]) |
                              (static_cast<uint16_t>(raw[raw.size() - 1]) << 8);

    if (crc_calc != crc_recv)
    {
        if (error) *error = "crc mismatch";
        return false;
    }

    out.header   = raw[0];
    out.sequence = raw[1];
    out.device   = raw[2];
    out.command  = raw[3];
    out.payload.assign(raw.begin() + 5, raw.begin() + 5 + length);

    return true;
}

void appendU16LE(std::vector<uint8_t>& out, uint16_t value)
{
    out.push_back(static_cast<uint8_t>(value & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void appendU32LE(std::vector<uint8_t>& out, uint32_t value)
{
    out.push_back(static_cast<uint8_t>(value & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

void appendI32LE(std::vector<uint8_t>& out, int32_t value)
{
    appendU32LE(out, static_cast<uint32_t>(value));
}

uint16_t readU16LE(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t readU32LE(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

int32_t readI32LE(const uint8_t* data)
{
    return static_cast<int32_t>(readU32LE(data));
}

void appendFloatLE(std::vector<uint8_t>& out, float value)
{
    static_assert(sizeof(float) == 4, "float must be 4 bytes");

    uint32_t raw = 0;
    std::memcpy(&raw, &value, sizeof(float));
    appendU32LE(out, raw);
}

float readFloatLE(const uint8_t* data)
{
    const uint32_t raw = readU32LE(data);
    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(float));
    return value;
}

}