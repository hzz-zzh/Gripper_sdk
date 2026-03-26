#ifndef GRIPPER_TRANSPORT_RS485_TRANSPORT_H
#define GRIPPER_TRANSPORT_RS485_TRANSPORT_H

#include "transport/i_transport.h"
#include <string>

namespace gripper
{
class Rs485Transport : public ITransport
{
public:
    Rs485Transport(const std::string& port_name, int baudrate);
    ~Rs485Transport() override;

    bool open() override;
    void close() override;
    bool isOpen() const override;

    int writeBytes(const uint8_t* data, std::size_t size) override;
    int readBytes(uint8_t* data, std::size_t size, int timeout_ms) override;

private:
    std::string port_name_;
    int baudrate_;
    int fd_;
};
}

#endif