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