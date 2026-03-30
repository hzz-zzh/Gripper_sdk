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

    // 预留给上层自定义初始工作位
    int32_t home_count = 0;
};

struct GripperInitializeConfig
{
    // 用低速朝机械限位搜索，建议填正值
    float search_speed_rpm = 5.0f;

    // +1 / -1，表示“朝哪个方向去找限位”
    int search_direction = +1;

    int poll_interval_ms = 20;
    int timeout_ms = 5000;

    // 连续满足以下判据，认为已经碰到机械限位
    float speed_epsilon_rpm = 0.5f;
    float current_threshold_a = 0.6f;
    int32_t position_epsilon_count = 5;
    int detect_consecutive_samples = 4;

    bool clear_fault_before_start = true;
    bool set_zero_after_detect = true;

    // 设零后，从机械限位回退一段安全距离
    // 这里填正值，实际实现会自动朝“远离限位”的方向回退
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