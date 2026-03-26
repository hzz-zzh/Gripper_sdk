#ifndef GRIPPER_TRANSPORT_I_TRANSPORT_H
#define GRIPPER_TRANSPORT_I_TRANSPORT_H

#include <cstddef>
#include <cstdint>

namespace gripper
{
class ITransport
{
public:
    virtual ~ITransport() = default;

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;

    virtual int writeBytes(const uint8_t* data, std::size_t size) = 0;
    virtual int readBytes(uint8_t* data, std::size_t size, int timeout_ms) = 0;
};
}

#endif