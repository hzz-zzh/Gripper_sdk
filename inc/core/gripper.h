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

struct GripperPositionProfile
{
    int32_t open_position_count = -3000;
    int32_t close_position_count = -53000;
};

enum class BrakeAction : uint8_t
{
    Release   = 0x00,
    Engage    = 0x01,
    ReadState = 0xFF
};

enum class Rs485BaudrateCode : uint8_t
{
    Baud921600 = 0,
    Baud460800 = 1,
    Baud115200 = 2,
    Baud57600  = 3,
    Baud38400  = 4,
    Baud19200  = 5,
    Baud9600   = 6
};

enum class CanBaudrateCode : uint8_t
{
    Baud1M   = 0,
    Baud500K = 1,
    Baud250K = 2,
    Baud125K = 3,
    Baud100K = 4
};

struct UserParameters
{
    uint16_t electrical_angle_offset = 0;
    uint16_t mechanical_angle_offset = 0;
    uint16_t current_offset_u = 0;
    uint16_t current_offset_v = 0;
    uint16_t current_offset_w = 0;

    uint8_t encoder_model = 0;
    bool invert_encoder_direction = false;
    bool enable_second_encoder = false;
    uint8_t speed_filter_coeff = 0;

    uint8_t device_address = 0x01;
    Rs485BaudrateCode rs485_baudrate = Rs485BaudrateCode::Baud115200;
    CanBaudrateCode can_baudrate = CanBaudrateCode::Baud1M;
    bool enable_canopen = false;

    uint16_t max_bus_voltage_0p01v = 0;
    uint8_t voltage_fault_delay_s = 0;

    uint16_t max_bus_current_0p01a = 0;
    uint8_t current_fault_delay_s = 0;

    uint8_t max_temperature_c = 0;
    uint8_t temperature_fault_delay_s = 0;
};

struct WritableUserParameters
{
    uint8_t encoder_model = 0;
    bool invert_encoder_direction = false;
    bool enable_second_encoder = false;
    uint8_t speed_filter_coeff = 0;

    uint8_t device_address = 0x01;
    Rs485BaudrateCode rs485_baudrate = Rs485BaudrateCode::Baud115200;
    CanBaudrateCode can_baudrate = CanBaudrateCode::Baud1M;
    bool enable_canopen = false;

    uint16_t max_bus_voltage_0p01v = 0;
    uint8_t voltage_fault_delay_s = 0;

    uint16_t max_bus_current_0p01a = 0;
    uint8_t current_fault_delay_s = 0;

    uint8_t max_temperature_c = 0;
    uint8_t temperature_fault_delay_s = 0;
};

struct MotionControlParameters
{
    float position_kp = 0.0f;
    float position_ki = 0.0f;

    uint32_t position_output_limit = 0;
    // 含义：位置模式最大速度，单位 0.01 rpm

    float speed_kp = 0.0f;
    float speed_ki = 0.0f;

    uint32_t speed_output_limit = 0;
    // 含义：速度/位置模式最大 Q 轴电流，单位 0.001 A
};

struct MotorHardwareParameters
{
    std::string motor_name;
    uint8_t pole_pairs = 0;

    float phase_resistance_ohm = 0.0f;
    float phase_inductance_mh = 0.0f;
    float torque_constant_nm = 0.0f;

    uint8_t gear_ratio = 0;
};

class Gripper
{
public:
    explicit Gripper(uint8_t device_address = 0x01);
    Gripper(uint8_t device_address, std::unique_ptr<ITransport> transport);

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
    bool moveToCountWithLimits(int32_t target_count,
                               float max_speed_rpm,
                               float max_current_amp,
                               RealtimeStatus* out = nullptr);
    bool moveByCount(int32_t delta_count, RealtimeStatus* out = nullptr);
    bool goHomeShortest(RealtimeStatus* out = nullptr);
    bool motorOff(RealtimeStatus* out = nullptr);

    bool reboot();
    bool setCurrentPositionAsZero(uint16_t& mechanical_offset);
    bool restoreDefaultParameters();
    bool brakeControl(BrakeAction action, uint8_t& brake_state);
    bool brakeRelease(uint8_t& brake_state);
    bool brakeEngage(uint8_t& brake_state);
    bool brakeReadState(uint8_t& brake_state);

    bool readUserParameters(UserParameters& out);
    bool writeUserParameters(const WritableUserParameters& in, UserParameters* out = nullptr);

    uint8_t deviceAddress() const;

    bool readMotionControlParameters(MotionControlParameters& out);
    bool writeMotionControlParametersTemp(const MotionControlParameters& in,
                                          MotionControlParameters* out = nullptr);
    bool writeMotionControlParametersSave(const MotionControlParameters& in,
                                          MotionControlParameters* out = nullptr);

    bool readMotorHardwareParameters(MotorHardwareParameters& out);
    bool writeMotorHardwareParameters(const MotorHardwareParameters& in,
                                      MotorHardwareParameters* out = nullptr);

    bool startEncoderCalibration(RealtimeStatus* out = nullptr);

    bool startEncoderCalibrationAndWait(int wait_ms,
                                        int poll_interval_ms,
                                        RealtimeStatus& out);

private:
    bool transact(protocol::Command cmd,
                  const std::vector<uint8_t>& payload,
                  protocol::Frame& response,
                  bool expect_response = true);

    bool readResponse(protocol::Frame& frame,
                      uint8_t expected_sequence,
                      protocol::Command expected_command);

    bool readResponseForDevice(protocol::Frame& frame,
                               uint8_t expected_sequence,
                               protocol::Command expected_command,
                               uint8_t expected_device_address);

    bool isExpectedResponseDevice(uint8_t response_device) const;
    static bool isExpectedResponseDeviceFor(uint8_t requested_device_address,
                                            uint8_t response_device);

    static bool parseRealtimePayload(const std::vector<uint8_t>& payload,
                                     RealtimeStatus& out,
                                     std::string& error);

    static bool parseUserParametersPayload(const std::vector<uint8_t>& payload,
                                           UserParameters& out,
                                           std::string& error);

    static std::vector<uint8_t> buildWritableUserParametersPayload(const WritableUserParameters& in);

    static bool parseMotionControlParametersPayload(const std::vector<uint8_t>& payload,
                                                    MotionControlParameters& out,
                                                    std::string& error);

    static std::vector<uint8_t> buildMotionControlParametersPayload(const MotionControlParameters& in);

    static bool parseMotorHardwareParametersPayload(const std::vector<uint8_t>& payload,
                                                    MotorHardwareParameters& out,
                                                    std::string& error);

    static std::vector<uint8_t> buildMotorHardwareParametersPayload(const MotorHardwareParameters& in);

private:
    std::unique_ptr<ITransport> transport_;
    uint8_t device_address_;
    uint8_t next_sequence_;
    int timeout_ms_;
    std::string last_error_;
};
}

#endif
