#include "core/gripper_device.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>

namespace gripper
{
namespace
{
constexpr int32_t kInternalOpenPositionCount = -53000;
constexpr int32_t kInternalClosePositionCount = -3000;
}

GripperDevice::GripperDevice(const GripperDeviceConfig& config)
    : config_(config),
      motor_(config.device_address),
      last_error_(),
      initialized_(false)
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

    initialized_ = false;
    last_error_.clear();
    return true;
}

void GripperDevice::disconnect()
{
    motor_.disconnect();
    initialized_ = false;
}

bool GripperDevice::isConnected() const
{
    return motor_.isConnected();
}

bool GripperDevice::isInitialized() const
{
    return initialized_;
}

void GripperDevice::invalidateInitialization()
{
    initialized_ = false;
}

const std::string& GripperDevice::lastError() const
{
    return last_error_;
}

bool GripperDevice::initialize(const GripperInitializeConfig& config,
                               GripperInitializeResult* out)
{
    if (config.search_direction != 1 && config.search_direction != -1)
    {
        last_error_ = "invalid initialize config: search_direction must be +1 or -1";
        return false;
    }

    if (config.search_speed_rpm <= 0.0f)
    {
        last_error_ = "invalid initialize config: search_speed_rpm must be > 0";
        return false;
    }

    if (config.poll_interval_ms <= 0)
    {
        last_error_ = "invalid initialize config: poll_interval_ms must be > 0";
        return false;
    }

    if (config.timeout_ms <= 0)
    {
        last_error_ = "invalid initialize config: timeout_ms must be > 0";
        return false;
    }

    if (config.detect_consecutive_samples <= 0)
    {
        last_error_ = "invalid initialize config: detect_consecutive_samples must be > 0";
        return false;
    }

    if (config.position_epsilon_count < 0)
    {
        last_error_ = "invalid initialize config: position_epsilon_count must be >= 0";
        return false;
    }

    initialized_ = false;

    if (out != nullptr)
    {
        *out = GripperInitializeResult{};
    }

    if (config.clear_fault_before_start)
    {
        uint8_t current_fault = 0;
        if (!motor_.clearFault(current_fault))
        {
            setLastErrorFromMotor();
            return false;
        }
    }

    const float cmd_speed_rpm =
        static_cast<float>(config.search_direction) * config.search_speed_rpm;

    if (!motor_.setSpeed(cmd_speed_rpm, 0, nullptr))
    {
        setLastErrorFromMotor();
        return false;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(config.timeout_ms);

    bool has_prev_count = false;
    int32_t prev_count = 0;
    int consecutive_hits = 0;
    RealtimeStatus latest{};

    while (std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(config.poll_interval_ms));

        if (!motor_.readRealtime(latest))
        {
            setLastErrorFromMotor();
            motor_.motorOff(nullptr);
            return false;
        }

        if (latest.fault_code != 0)
        {
            motor_.motorOff(nullptr);
            last_error_ = "fault occurred during initialize";
            return false;
        }

        int32_t delta_count = 0;
        if (has_prev_count)
        {
            delta_count = std::abs(latest.multi_turn_count - prev_count);
        }

        prev_count = latest.multi_turn_count;
        has_prev_count = true;

        const bool speed_small =
            std::abs(latest.speed_rpm) <= config.speed_epsilon_rpm;

        const bool current_high =
            std::abs(latest.q_current_amp) >= config.current_threshold_a;

        const bool position_locked =
            has_prev_count && (delta_count <= config.position_epsilon_count);

        if (speed_small && current_high && position_locked)
        {
            ++consecutive_hits;
        }
        else
        {
            consecutive_hits = 0;
        }

        if (consecutive_hits >= config.detect_consecutive_samples)
        {
            if (!motor_.motorOff(nullptr))
            {
                setLastErrorFromMotor();
                return false;
            }

            if (out != nullptr)
            {
                out->limit_detected = true;
                out->detect_samples = consecutive_hits;
                out->limit_count_before_zero = latest.multi_turn_count;
            }

            uint16_t mechanical_offset = 0;
            if (config.set_zero_after_detect)
            {
                if (!motor_.setCurrentPositionAsZero(mechanical_offset))
                {
                    setLastErrorFromMotor();
                    return false;
                }

                if (out != nullptr)
                {
                    out->zero_set = true;
                    out->mechanical_offset = mechanical_offset;
                }
            }

            if (config.backoff_count_after_zero > 0)
            {
                const int32_t backoff_delta =
                    -config.search_direction * config.backoff_count_after_zero;

                if (!motor_.moveByCount(backoff_delta, &latest))
                {
                    setLastErrorFromMotor();
                    return false;
                }

                if (out != nullptr)
                {
                    out->backoff_done = true;
                }
            }

            if (!motor_.readRealtime(latest))
            {
                setLastErrorFromMotor();
                return false;
            }

            if (out != nullptr)
            {
                out->final_status = latest;
            }

            initialized_ = true;
            last_error_.clear();
            return true;
        }
    }

    motor_.motorOff(nullptr);
    last_error_ = "initialize timeout";
    return false;
}

bool GripperDevice::moveToPosition(int32_t target_position, RealtimeStatus* out)
{
    if (!initialized_)
    {
        last_error_ = "gripper not initialized";
        return false;
    }

    if (!motor_.moveToCount(target_position, out))
    {
        setLastErrorFromMotor();
        return false;
    }

    last_error_.clear();
    return true;
}

bool GripperDevice::moveRelative(int32_t delta_position, RealtimeStatus* out)
{
    if (!initialized_)
    {
        last_error_ = "gripper not initialized";
        return false;
    }

    if (!motor_.moveByCount(delta_position, out))
    {
        setLastErrorFromMotor();
        return false;
    }

    last_error_.clear();
    return true;
}

bool GripperDevice::moveToPercent(float percent, RealtimeStatus* out)
{
    if (!initialized_)
    {
        last_error_ = "gripper not initialized";
        return false;
    }

    if (kInternalOpenPositionCount == kInternalClosePositionCount)
    {
        last_error_ = "invalid internal gripper range";
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
    return moveToPosition(kInternalOpenPositionCount, out);
}

bool GripperDevice::close(RealtimeStatus* out)
{
    return moveToPosition(kInternalClosePositionCount, out);
}

int32_t GripperDevice::percentToCount(float percent) const
{
    const float clamped = std::clamp(percent, 0.0f, 100.0f);
    const float ratio = clamped / 100.0f;

    const float count =
        static_cast<float>(kInternalClosePositionCount) +
        ratio * static_cast<float>(kInternalOpenPositionCount - kInternalClosePositionCount);

    return static_cast<int32_t>(std::lround(count));
}

float GripperDevice::countToPercent(int32_t count) const
{
    const int32_t span = kInternalOpenPositionCount - kInternalClosePositionCount;
    if (span == 0)
    {
        return 0.0f;
    }

    const float ratio =
        static_cast<float>(count - kInternalClosePositionCount) /
        static_cast<float>(span);

    return std::clamp(ratio * 100.0f, 0.0f, 100.0f);
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