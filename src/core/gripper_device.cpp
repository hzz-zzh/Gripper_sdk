#include "core/gripper_device.h"

#include <algorithm>
#include <cmath>

namespace gripper
{
GripperDevice::GripperDevice(const GripperDeviceConfig& config)
    : config_(config),
      motor_(config.device_address),
      last_error_()
{
    motor_.setTimeoutMs(config_.timeout_ms);
}

bool GripperDevice::connect()
{
    if (!motor_.connect(config_.port_name, config_.baudrate))
    {
        setLastErrorFromMotor();
        return false;
    }

    last_error_.clear();
    return true;
}

void GripperDevice::disconnect()
{
    motor_.disconnect();
}

bool GripperDevice::isConnected() const
{
    return motor_.isConnected();
}

const std::string& GripperDevice::lastError() const
{
    return last_error_;
}

bool GripperDevice::moveToPercent(float percent, RealtimeStatus* out)
{
    if (config_.fully_open_count == config_.fully_close_count)
    {
        last_error_ = "invalid gripper config: fully_open_count equals fully_close_count";
        return false;
    }

    const int32_t target_count = percentToCount(percent);
    if (!motor_.moveToCount(target_count, out))
    {
        setLastErrorFromMotor();
        return false;
    }

    last_error_.clear();
    return true;
}

bool GripperDevice::open(RealtimeStatus* out)
{
    return moveToPercent(100.0f, out);
}

bool GripperDevice::close(RealtimeStatus* out)
{
    return moveToPercent(0.0f, out);
}

bool GripperDevice::stop(RealtimeStatus* out)
{
    if (!motor_.motorOff(out))
    {
        setLastErrorFromMotor();
        return false;
    }

    last_error_.clear();
    return true;
}

int32_t GripperDevice::percentToCount(float percent) const
{
    const float clamped = std::clamp(percent, 0.0f, 100.0f);
    const float ratio = clamped / 100.0f;

    const float count =
        static_cast<float>(config_.fully_close_count) +
        ratio * static_cast<float>(config_.fully_open_count - config_.fully_close_count);

    return static_cast<int32_t>(std::lround(count));
}

float GripperDevice::countToPercent(int32_t count) const
{
    const int32_t span = config_.fully_open_count - config_.fully_close_count;
    if (span == 0)
    {
        return 0.0f;
    }

    const float ratio =
        static_cast<float>(count - config_.fully_close_count) /
        static_cast<float>(span);

    return std::clamp(ratio * 100.0f, 0.0f, 100.0f);
}

bool GripperDevice::readRealtime(RealtimeStatus& out)
{
    if (!motor_.readRealtime(out))
    {
        setLastErrorFromMotor();
        return false;
    }

    last_error_.clear();
    return true;
}

Gripper& GripperDevice::motor()
{
    return motor_;
}

void GripperDevice::setLastErrorFromMotor()
{
    last_error_ = motor_.lastError();
}
}