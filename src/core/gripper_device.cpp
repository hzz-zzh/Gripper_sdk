#include "core/gripper_device.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <utility>

namespace gripper
{
namespace
{
constexpr double kEncoderCountPerRev = 16384.0;
constexpr double kReducerRatio = 24.0;
constexpr double kTotalCountPerTurbineRev = kEncoderCountPerRev * kReducerRatio;

constexpr double kLinkLengthMm = 60.0;
constexpr double kAlphaBreakDeg = 27.4275788;
constexpr double kAlphaOffsetDeg = 10.56397759;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegPerCount = 360.0 / kTotalCountPerTurbineRev;
constexpr double kAlphaMinDeg = 0.0;
constexpr double kAlphaMaxDeg = kAlphaBreakDeg + kAlphaOffsetDeg;

inline double degToRad(double deg)
{
    return deg * kPi / 180.0;
}

inline double radToDeg(double rad)
{
    return rad * 180.0 / kPi;
}

inline double clampUnit(double x)
{
    if (x < -1.0)
    {
        return -1.0;
    }
    if (x > 1.0)
    {
        return 1.0;
    }
    return x;
}

inline double maxOpeningFromFormula()
{
    return 2.0 * kLinkLengthMm *
           (std::sin(degToRad(kAlphaBreakDeg)) +
            std::sin(degToRad(kAlphaOffsetDeg)));
}
} // namespace

GripperDevice::GripperDevice(const GripperDeviceConfig& config)
    : config_(config),
      motor_(config.device_address),
      last_error_(),
      initialized_(false)
{
    motor_.setTimeoutMs(config_.timeout_ms);
}

GripperDevice::GripperDevice(const GripperDeviceConfig& config,
                             std::unique_ptr<ITransport> transport)
    : config_(config),
      motor_(config.device_address, std::move(transport)),
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

    if (config.search_speed_mm_s <= 0.0f)
    {
        last_error_ = "invalid initialize config: search_speed_mm_s must be > 0";
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

    if (config.speed_epsilon_mm_s < 0.0f)
    {
        last_error_ = "invalid initialize config: speed_epsilon_mm_s must be >= 0";
        return false;
    }

    if (config.position_epsilon_mm < 0.0f)
    {
        last_error_ = "invalid initialize config: position_epsilon_mm must be >= 0";
        return false;
    }

    if (config.backoff_after_zero_mm < 0.0f)
    {
        last_error_ = "invalid initialize config: backoff_after_zero_mm must be >= 0";
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

    const float search_speed_rpm =
        openingSpeedMmSToMotorRpmConservative(config.search_speed_mm_s);

    const float cmd_speed_rpm =
        static_cast<float>(config.search_direction) * search_speed_rpm;

    if (!motor_.setSpeed(cmd_speed_rpm, 0, nullptr))
    {
        setLastErrorFromMotor();
        return false;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(config.timeout_ms);

    bool has_prev_status = false;
    RealtimeStatus prev_status{};
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

        float delta_opening_mm = std::numeric_limits<float>::max();
        if (has_prev_status)
        {
            const float prev_opening_mm = countToOpeningMm(prev_status.multi_turn_count);
            const float latest_opening_mm = countToOpeningMm(latest.multi_turn_count);
            delta_opening_mm = std::abs(latest_opening_mm - prev_opening_mm);
        }

        const float opening_speed_mm_s =
            motorRpmToOpeningSpeedMmS(latest.speed_rpm, latest.multi_turn_count);

        const bool speed_small = std::abs(opening_speed_mm_s) <= config.speed_epsilon_mm_s;
        const bool current_high = std::abs(latest.q_current_amp) >= config.current_threshold_a;
        const bool position_locked = has_prev_status &&
                                     (delta_opening_mm <= config.position_epsilon_mm);

        if (speed_small && current_high && position_locked)
        {
            ++consecutive_hits;
        }
        else
        {
            consecutive_hits = 0;
        }

        prev_status = latest;
        has_prev_status = true;

        if (consecutive_hits >= config.detect_consecutive_samples)
        {
            if (out != nullptr)
            {
                out->limit_detected = true;
                out->detect_samples = consecutive_hits;
                out->limit_opening_mm_before_zero = countToOpeningMm(latest.multi_turn_count);
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

                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            if (config.backoff_after_zero_mm > 0.0f)
            {
                const int32_t backoff_count_mag =
                    openingMmToBackoffDeltaCount(config.backoff_after_zero_mm, 0);
                const int32_t backoff_delta = -config.search_direction * backoff_count_mag;

                RealtimeStatus before_backoff{};
                if (!motor_.readRealtime(before_backoff))
                {
                    setLastErrorFromMotor();
                    return false;
                }

                if (!motor_.moveByCount(backoff_delta, &latest))
                {
                    setLastErrorFromMotor();
                    return false;
                }

                const auto backoff_deadline =
                    std::chrono::steady_clock::now() + std::chrono::milliseconds(800);

                bool backoff_started = false;
                while (std::chrono::steady_clock::now() < backoff_deadline)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));

                    RealtimeStatus now{};
                    if (!motor_.readRealtime(now))
                    {
                        setLastErrorFromMotor();
                        return false;
                    }

                    latest = now;

                    if (latest.fault_code != 0)
                    {
                        last_error_ = "fault occurred during homing backoff";
                        return false;
                    }

                    if (std::abs(now.multi_turn_count - before_backoff.multi_turn_count) > 50)
                    {
                        backoff_started = true;
                        break;
                    }
                }

                if (out != nullptr && backoff_started)
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
                convertRealtimeToStatus(latest, out->final_status);
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

bool GripperDevice::moveToOpeningMm(float target_opening_mm, GripperStatus* out)
{
    if (!initialized_)
    {
        last_error_ = "gripper not initialized";
        return false;
    }

    int32_t target_count = 0;
    if (!openingMmToCount(target_opening_mm, target_count))
    {
        return false;
    }

    RealtimeStatus realtime{};
    if (!motor_.moveToCount(target_count, out ? &realtime : nullptr))
    {
        setLastErrorFromMotor();
        return false;
    }

    if (out != nullptr)
    {
        convertRealtimeToStatus(realtime, *out);
    }

    last_error_.clear();
    return true;
}

bool GripperDevice::moveToOpeningMmWithLimits(float target_opening_mm,
                                              float max_speed_mm_s,
                                              float max_current_amp,
                                              GripperStatus* out)
{
    if (!initialized_)
    {
        last_error_ = "gripper not initialized";
        return false;
    }

    int32_t target_count = 0;
    if (!openingMmToCount(target_opening_mm, target_count))
    {
        return false;
    }

    float max_speed_rpm = 0.0f;
    if (max_speed_mm_s > 0.0f)
    {
        RealtimeStatus current{};
        if (!motor_.readRealtime(current))
        {
            setLastErrorFromMotor();
            return false;
        }

        max_speed_rpm =
            openingSpeedMmSToMotorRpm(max_speed_mm_s, current.multi_turn_count);
    }

    RealtimeStatus realtime{};
    if (!motor_.moveToCountWithLimits(target_count,
                                      max_speed_rpm,
                                      max_current_amp,
                                      out ? &realtime : nullptr))
    {
        setLastErrorFromMotor();
        return false;
    }

    if (out != nullptr)
    {
        convertRealtimeToStatus(realtime, *out);
    }

    last_error_.clear();
    return true;
}

bool GripperDevice::open(GripperStatus* out)
{
    return moveToOpeningMm(maxOpeningMm(), out);
}

bool GripperDevice::close(GripperStatus* out)
{
    return moveToOpeningMm(minOpeningMm(), out);
}

bool GripperDevice::stop(GripperStatus* out)
{
    RealtimeStatus realtime{};
    if (!motor_.motorOff(out ? &realtime : nullptr))
    {
        setLastErrorFromMotor();
        return false;
    }

    if (out != nullptr)
    {
        convertRealtimeToStatus(realtime, *out);
    }

    last_error_.clear();
    return true;
}

bool GripperDevice::readStatus(GripperStatus& out)
{
    RealtimeStatus realtime{};
    if (!motor_.readRealtime(realtime))
    {
        setLastErrorFromMotor();
        return false;
    }

    convertRealtimeToStatus(realtime, out);
    last_error_.clear();
    return true;
}

bool GripperDevice::reboot()
{
    if (!motor_.reboot())
    {
        setLastErrorFromMotor();
        return false;
    }

    initialized_ = false;
    last_error_.clear();
    return true;
}

float GripperDevice::minOpeningMm() const
{
    return 0.0f;
}

float GripperDevice::maxOpeningMm() const
{
    return static_cast<float>(maxOpeningFromFormula());
}

double GripperDevice::countToTurbineAngleDeg(int32_t count) const
{
    return -static_cast<double>(count) * kDegPerCount;
}

double GripperDevice::turbineAngleDegToOpeningMm(double alpha_deg) const
{
    const double alpha = std::clamp(alpha_deg, kAlphaMinDeg, kAlphaMaxDeg);

    return 2.0 * kLinkLengthMm *
           (std::sin(degToRad(kAlphaBreakDeg - alpha)) +
            std::sin(degToRad(kAlphaOffsetDeg)));
}

float GripperDevice::countToOpeningMm(int32_t count) const
{
    const double opening = turbineAngleDegToOpeningMm(countToTurbineAngleDeg(count));
    return static_cast<float>(std::clamp(opening,
                                         static_cast<double>(minOpeningMm()),
                                         static_cast<double>(maxOpeningMm())));
}

double GripperDevice::openingSpeedScaleMmPerSecPerRpm(double alpha_deg) const
{
    const double alpha = std::clamp(alpha_deg, kAlphaMinDeg, kAlphaMaxDeg);
    return (kPi / 6.0) * std::cos(degToRad(kAlphaBreakDeg - alpha));
}

float GripperDevice::motorRpmToOpeningSpeedMmS(float motor_speed_rpm, int32_t count) const
{
    const double alpha_deg = countToTurbineAngleDeg(count);
    const double scale = openingSpeedScaleMmPerSecPerRpm(alpha_deg);
    return static_cast<float>(scale * static_cast<double>(motor_speed_rpm));
}

float GripperDevice::openingSpeedMmSToMotorRpm(float opening_speed_mm_s,
                                               int32_t reference_count) const
{
    const double alpha_deg = countToTurbineAngleDeg(reference_count);
    const double scale = openingSpeedScaleMmPerSecPerRpm(alpha_deg);
    const double safe_scale = std::max(scale, 1e-6);
    return static_cast<float>(static_cast<double>(opening_speed_mm_s) / safe_scale);
}

float GripperDevice::openingSpeedMmSToMotorRpmConservative(float opening_speed_mm_s) const
{
    if (opening_speed_mm_s <= 0.0f)
    {
        return 0.0f;
    }

    constexpr double kMaxScale = kPi / 6.0;
    return static_cast<float>(static_cast<double>(opening_speed_mm_s) / kMaxScale);
}

bool GripperDevice::openingMmToCount(float opening_mm, int32_t& out_count)
{
    const double min_mm = static_cast<double>(minOpeningMm());
    const double max_mm = static_cast<double>(maxOpeningMm());
    const double target_mm = static_cast<double>(opening_mm);

    if (target_mm < min_mm - 1e-6 || target_mm > max_mm + 1e-6)
    {
        last_error_ = "target opening_mm is outside the valid geometry range";
        return false;
    }

    const double s = target_mm / (2.0 * kLinkLengthMm) -
                     std::sin(degToRad(kAlphaOffsetDeg));

    const double alpha_deg =
        kAlphaBreakDeg - radToDeg(std::asin(clampUnit(s)));

    const double alpha_clamped = std::clamp(alpha_deg, kAlphaMinDeg, kAlphaMaxDeg);
    out_count = static_cast<int32_t>(std::lround(-alpha_clamped / kDegPerCount));

    last_error_.clear();
    return true;
}

int32_t GripperDevice::openingMmToBackoffDeltaCount(float delta_mm, int32_t reference_count) const
{
    if (!(delta_mm > 0.0f))
    {
        return 0;
    }

    const float c0 = countToOpeningMm(reference_count);
    const float c1 = countToOpeningMm(reference_count + 1);
    const float c2 = countToOpeningMm(reference_count - 1);

    const float mm_per_count =
        std::max({std::abs(c1 - c0), std::abs(c2 - c0), 1e-6f});

    return static_cast<int32_t>(std::ceil(delta_mm / mm_per_count));
}

bool GripperDevice::convertRealtimeToStatus(const RealtimeStatus& in, GripperStatus& out) const
{
    out.opening_mm = countToOpeningMm(in.multi_turn_count);
    out.opening_speed_mm_s = motorRpmToOpeningSpeedMmS(in.speed_rpm, in.multi_turn_count);
    out.q_current_amp = in.q_current_amp;
    out.bus_voltage_v = in.bus_voltage_v;
    out.bus_current_a = in.bus_current_a;
    out.temperature_c = in.temperature_c;
    out.run_state = in.run_state;
    out.motor_enabled = in.motor_enabled;
    out.fault_code = in.fault_code;
    return true;
}

void GripperDevice::setLastErrorFromMotor()
{
    last_error_ = motor_.lastError();
}
} // namespace gripper