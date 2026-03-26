#ifndef GRIPPER_CORE_GRIPPER_H
#define GRIPPER_CORE_GRIPPER_H

#include "protocol/protocol.h"
#include "transport/i_transport.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace gripper
{
struct VersionInfo
{
    uint16_t boot_version = 0;
    uint16_t app_version = 0;
    uint16_t hardware_model = 0;
    uint8_t rs485_custom_version = 0;
    uint8_t rs485_modbus_version = 0;
    uint8_t can_custom_version = 0;
    uint8_t canopen_version = 0;
    std::array<uint8_t, 12> uid{};
};

struct RealtimeStatus
{
    uint16_t single_turn_raw = 0;
    float single_turn_deg = 0.0f;

    int32_t multi_turn_count = 0;
    float multi_turn_deg = 0.0f;

    int32_t speed_raw = 0;
    float speed_rpm = 0.0f;

    int32_t q_current_raw = 0;
    float q_current_amp = 0.0f;

    uint16_t bus_voltage_raw = 0;
    float bus_voltage_v = 0.0f;

    uint16_t bus_current_raw = 0;
    float bus_current_a = 0.0f;

    uint8_t temperature_c = 0;
    uint8_t run_state = 0;
    bool motor_enabled = false;
    uint8_t fault_code = 0;
};

enum class BrakeAction : uint8_t
{
    Release   = 0x00,
    Engage    = 0x01,
    ReadState = 0xFF
};

class Gripper
{
public:
    explicit Gripper(uint8_t device_address = 0x01);

    bool connect(const std::string& port_name, int baudrate = 115200);
    void disconnect();
    bool isConnected() const;

    void setTimeoutMs(int timeout_ms);
    const std::string& lastError() const;

    bool readVersion(VersionInfo& out);
    bool readRealtime(RealtimeStatus& out);
    bool clearFault(uint8_t& current_fault);

    bool setQCurrent(float current_amp,
                     uint32_t slope_milliamp_per_sec = 0,
                     RealtimeStatus* out = nullptr);

    bool setSpeed(float rpm,
                  uint32_t accel_0p01rpm_per_sec = 0,
                  RealtimeStatus* out = nullptr);

    bool moveToCount(int32_t target_count, RealtimeStatus* out = nullptr);
    bool moveByCount(int32_t delta_count, RealtimeStatus* out = nullptr);
    bool goHomeShortest(RealtimeStatus* out = nullptr);
    bool motorOff(RealtimeStatus* out = nullptr);

    // 第一批补充命令
    bool reboot();
    bool setCurrentPositionAsZero(uint16_t& mechanical_offset);
    bool restoreDefaultParameters();
    bool brakeControl(BrakeAction action, uint8_t& brake_state);
    bool brakeRelease(uint8_t& brake_state);
    bool brakeEngage(uint8_t& brake_state);
    bool brakeReadState(uint8_t& brake_state);

private:
    bool transact(protocol::Command cmd,
                  const std::vector<uint8_t>& payload,
                  protocol::Frame& response,
                  bool expect_response = true);

    bool readResponse(protocol::Frame& frame);
    static bool parseRealtimePayload(const std::vector<uint8_t>& payload,
                                     RealtimeStatus& out,
                                     std::string& error);

private:
    std::unique_ptr<ITransport> transport_;
    uint8_t device_address_;
    uint8_t next_sequence_;
    int timeout_ms_;
    std::string last_error_;
};
}

#endif