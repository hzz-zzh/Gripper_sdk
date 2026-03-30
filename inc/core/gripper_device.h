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

    // 夹爪开合映射
    // 约定：0% = fully_close_count, 100% = fully_open_count
    int32_t fully_open_count = 0;
    int32_t fully_close_count = 16384;

    // 预留给后续 home() 使用
    int32_t home_count = 0;
};

struct GripperHomingConfig
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

struct GripperHomingResult
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

    const std::string& lastError() const;

    bool moveToPercent(float percent, RealtimeStatus* out = nullptr);
    bool open(RealtimeStatus* out = nullptr);
    bool close(RealtimeStatus* out = nullptr);
    bool stop(RealtimeStatus* out = nullptr);

    bool homing(const GripperHomingConfig& config,
                GripperHomingResult* out = nullptr);

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
};
}

#endif