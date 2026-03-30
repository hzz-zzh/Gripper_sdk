#ifndef GRIPPER_CORE_GRIPPER_DEVICE_H
#define GRIPPER_CORE_GRIPPER_DEVICE_H

#include "core/gripper.h"

#include <cstdint>
#include <string>

namespace gripper
{
struct GripperDeviceConfig
{
    std::string port_name = "/dev/ttyACM0";
    int baudrate = 115200;
    uint8_t device_address = 0x01;
    int timeout_ms = 200;
};

struct GripperInitializeConfig
{
    float search_speed_rpm = 5.0f;
    int search_direction = +1;

    int poll_interval_ms = 20;
    int timeout_ms = 5000;

    float speed_epsilon_rpm = 0.5f;
    float current_threshold_a = 0.6f;
    int32_t position_epsilon_count = 5;
    int detect_consecutive_samples = 4;

    bool clear_fault_before_start = true;
    bool set_zero_after_detect = true;

    int32_t backoff_count_after_zero = 200;
};

struct GripperInitializeResult
{
    bool limit_detected = false;
    bool zero_set = false;
    bool backoff_done = false;

    int detect_samples = 0;
    int32_t limit_count_before_zero = 0;
    uint16_t mechanical_offset = 0;

    RealtimeStatus final_status{};
};

class GripperDevice
{
public:
    explicit GripperDevice(const GripperDeviceConfig& config = {});

    bool connect();
    void disconnect();
    bool isConnected() const;

    bool initialize(const GripperInitializeConfig& config,
                    GripperInitializeResult* out = nullptr);
    bool isInitialized() const;
    void invalidateInitialization();

    const std::string& lastError() const;

    bool moveToPosition(int32_t target_position, RealtimeStatus* out = nullptr);
    bool moveToPositionWithLimits(int32_t target_position,
                                  float max_speed_rpm = 0.0f,
                                  float max_current_amp = 0.0f,
                                  RealtimeStatus* out = nullptr);
    bool moveRelative(int32_t delta_position, RealtimeStatus* out = nullptr);

    bool moveToPercent(float percent, RealtimeStatus* out = nullptr);
    bool open(RealtimeStatus* out = nullptr);
    bool close(RealtimeStatus* out = nullptr);
    bool stop(RealtimeStatus* out = nullptr);

    int32_t percentToCount(float percent) const;
    float countToPercent(int32_t count) const;

    bool readRealtime(RealtimeStatus& out);
    Gripper& motor();

private:
    void setLastErrorFromMotor();

private:
    GripperDeviceConfig config_;
    Gripper motor_;
    std::string last_error_;
    bool initialized_;
};
}

#endif