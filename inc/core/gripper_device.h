#ifndef GRIPPER_CORE_GRIPPER_DEVICE_H
#define GRIPPER_CORE_GRIPPER_DEVICE_H

#include "core/gripper.h"

#include <cstdint>
#include <memory>
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

struct GripperStatus
{
    float opening_mm = 0.0f;
    float opening_speed_mm_s = 0.0f;
    float q_current_amp = 0.0f;
    float bus_voltage_v = 0.0f;
    float bus_current_a = 0.0f;
    uint8_t temperature_c = 0;
    uint8_t run_state = 0;
    bool motor_enabled = false;
    uint8_t fault_code = 0;
};

struct GripperInitializeConfig
{
    float search_speed_mm_s = 50.0f;
    int search_direction = +1;

    int poll_interval_ms = 20;
    int timeout_ms = 5000;

    float speed_epsilon_mm_s = 0.25f;
    float current_threshold_a = 0.6f;
    float position_epsilon_mm = 0.05f;
    int detect_consecutive_samples = 4;

    bool clear_fault_before_start = true;
    bool set_zero_after_detect = true;

    float backoff_after_zero_mm = 5.0f;
};

struct GripperInitializeResult
{
    bool limit_detected = false;
    bool zero_set = false;
    bool backoff_done = false;

    int detect_samples = 0;
    float limit_opening_mm_before_zero = 0.0f;
    uint16_t mechanical_offset = 0;

    GripperStatus final_status{};
};

class GripperDevice
{
public:
    explicit GripperDevice(const GripperDeviceConfig& config = {});
    GripperDevice(const GripperDeviceConfig& config, std::unique_ptr<ITransport> transport);

    bool connect();
    void disconnect();
    bool isConnected() const;

    bool initialize(const GripperInitializeConfig& config,
                    GripperInitializeResult* out = nullptr);
    bool isInitialized() const;
    void invalidateInitialization();

    const std::string& lastError() const;

    bool moveToOpeningMm(float target_opening_mm, GripperStatus* out = nullptr);
    bool moveToOpeningMmWithLimits(float target_opening_mm,
                                   float max_speed_mm_s = 0.0f,
                                   float max_current_amp = 0.0f,
                                   GripperStatus* out = nullptr);

    bool open(GripperStatus* out = nullptr);
    bool close(GripperStatus* out = nullptr);
    bool stop(GripperStatus* out = nullptr);

    bool readStatus(GripperStatus& out);
    bool reboot();

    float minOpeningMm() const;
    float maxOpeningMm() const;

private:
    double countToTurbineAngleDeg(int32_t count) const;
    double turbineAngleDegToOpeningMm(double alpha_deg) const;
    float countToOpeningMm(int32_t count) const;

    double openingSpeedScaleMmPerSecPerRpm(double alpha_deg) const;
    float motorRpmToOpeningSpeedMmS(float motor_speed_rpm, int32_t count) const;
    float openingSpeedMmSToMotorRpm(float opening_speed_mm_s, int32_t reference_count) const;
    float openingSpeedMmSToMotorRpmConservative(float opening_speed_mm_s) const;

    bool openingMmToCount(float opening_mm, int32_t& out_count);
    int32_t openingMmToBackoffDeltaCount(float delta_mm, int32_t reference_count) const;
    bool convertRealtimeToStatus(const RealtimeStatus& in, GripperStatus& out) const;
    void setLastErrorFromMotor();

private:
    GripperDeviceConfig config_;
    Gripper motor_;
    std::string last_error_;
    bool initialized_;
};
}

#endif