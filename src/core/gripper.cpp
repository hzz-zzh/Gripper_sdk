#include "core/gripper.h"

namespace gripper
{
Gripper::Gripper()
    : connected_(false), enabled_(false), current_position_(0.0f)
{
}

bool Gripper::connect(const std::string& port, int baudrate)
{
    (void)port;
    (void)baudrate;

    connected_ = true;
    return true;
}

void Gripper::disconnect()
{
    connected_ = false;
    enabled_ = false;
}

bool Gripper::enable()
{
    if (!connected_)
    {
        return false;
    }

    enabled_ = true;
    return true;
}

bool Gripper::move(float position)
{
    if (!connected_ || !enabled_)
    {
        return false;
    }

    current_position_ = position;
    return true;
}

std::string Gripper::getStatus() const
{
    if (!connected_)
    {
        return "disconnected";
    }

    if (!enabled_)
    {
        return "connected but disabled";
    }

    return "connected and enabled, position = " + std::to_string(current_position_);
}
}