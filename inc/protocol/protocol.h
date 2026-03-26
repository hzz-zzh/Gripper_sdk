#ifndef GRIPPER_PROTOCOL_PROTOCOL_H
#define GRIPPER_PROTOCOL_PROTOCOL_H

#include <cstdint>
#include <string>
#include <vector>

namespace gripper::protocol
{
constexpr uint8_t kMasterHeader = 0xAE;
constexpr uint8_t kSlaveHeader  = 0xAC;

enum class Command : uint8_t
{
    Reboot        = 0x00,
    ReadVersion   = 0x0A,
    ReadRealtime  = 0x0B,
    ClearFault    = 0x0F,

    ReadUserParams = 0x10,
    WriteUserParams = 0x11,

    SetZeroPoint   = 0x1D,
    EncoderCalib   = 0x1E,
    RestoreDefault = 0x1F,

    WriteQCurrent = 0x20,
    WriteSpeed    = 0x21,
    MoveAbsolute  = 0x22,
    MoveRelative  = 0x23,
    GoHomeShortest = 0x24,

    BrakeControl  = 0x2E,
    MotorOff      = 0x2F
};

struct Frame
{
    uint8_t header   = 0;
    uint8_t sequence = 0;
    uint8_t device   = 0;
    uint8_t command  = 0;
    std::vector<uint8_t> payload;
};

std::vector<uint8_t> buildFrame(uint8_t header,
                                uint8_t sequence,
                                uint8_t device,
                                uint8_t command,
                                const std::vector<uint8_t>& payload);

std::vector<uint8_t> buildRequest(uint8_t sequence,
                                  uint8_t device,
                                  Command command,
                                  const std::vector<uint8_t>& payload);

bool parseFrame(const std::vector<uint8_t>& raw, Frame& out, std::string* error = nullptr);

void appendU16LE(std::vector<uint8_t>& out, uint16_t value);
void appendU32LE(std::vector<uint8_t>& out, uint32_t value);
void appendI32LE(std::vector<uint8_t>& out, int32_t value);

uint16_t readU16LE(const uint8_t* data);
uint32_t readU32LE(const uint8_t* data);
int32_t  readI32LE(const uint8_t* data);
}

#endif