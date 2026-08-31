#include "core/gripper_device.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
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
constexpr double kSoftwareMaxOpeningMm = 105.0;

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

inline double openingFormulaScale()
{
    return kSoftwareMaxOpeningMm / maxOpeningFromFormula();
}

inline bool baudrateToCode(int baudrate, Rs485BaudrateCode& code)
{
    switch (baudrate)
    {
    case 921600:
        code = Rs485BaudrateCode::Baud921600;
        return true;
    case 460800:
        code = Rs485BaudrateCode::Baud460800;
        return true;
    case 115200:
        code = Rs485BaudrateCode::Baud115200;
        return true;
    case 57600:
        code = Rs485BaudrateCode::Baud57600;
        return true;
    case 38400:
        code = Rs485BaudrateCode::Baud38400;
        return true;
    case 19200:
        code = Rs485BaudrateCode::Baud19200;
        return true;
    case 9600:
        code = Rs485BaudrateCode::Baud9600;
        return true;
    default:
        return false;
    }
}

inline bool isReadTimeoutError(const std::string& error)
{
    return error == "read timeout";
}

inline std::string faultCodeToText(uint8_t fault_code)
{
    if (fault_code == 0)
    {
        return "no fault";
    }

    std::string text;
    if ((fault_code & (1u << 0)) != 0u)
    {
        text += "voltage fault, ";
    }
    if ((fault_code & (1u << 1)) != 0u)
    {
        text += "current fault, ";
    }
    if ((fault_code & (1u << 2)) != 0u)
    {
        text += "temperature fault, ";
    }
    if ((fault_code & (1u << 3)) != 0u)
    {
        text += "encoder fault, ";
    }
    if ((fault_code & (1u << 6)) != 0u)
    {
        text += "hardware fault, ";
    }
    if ((fault_code & (1u << 7)) != 0u)
    {
        text += "software fault, ";
    }

    if (!text.empty())
    {
        text.resize(text.size() - 2);
    }
    return text;
}

inline std::string makeFaultContextError(const char* context, uint8_t fault_code)
{
    return std::string(context) + ": " + faultCodeToText(fault_code);
}

inline bool homingDebugEnabled()
{
    const char* value = std::getenv("GRIPPER_HOMING_DEBUG");
    return value != nullptr && value[0] != '\0' && value[0] != '0';
}

void homingDebugLog(const char* format, ...)
{
    if (!homingDebugEnabled())
    {
        return;
    }

    std::fprintf(stderr, "[gripper homing] ");

    va_list args;
    va_start(args, format);
    std::vfprintf(stderr, format, args);
    va_end(args);

    std::fprintf(stderr, "\n");
}

void homingDebugStatus(const char* label, const RealtimeStatus& status)
{
    homingDebugLog("%s count=%ld speed_rpm=%.3f q_current=%.3f fault=0x%02X run_state=%u enabled=%d",
                   label,
                   static_cast<long>(status.multi_turn_count),
                   static_cast<double>(status.speed_rpm),
                   static_cast<double>(status.q_current_amp),
                   static_cast<unsigned>(status.fault_code),
                   static_cast<unsigned>(status.run_state),
                   status.motor_enabled ? 1 : 0);
}

inline bool motionDebugEnabled()
{
    const char* value = std::getenv("GRIPPER_MOTION_DEBUG");
    return value != nullptr && value[0] != '\0' && value[0] != '0';
}

void motionDebugLog(const char* format, ...)
{
    if (!motionDebugEnabled())
    {
        return;
    }

    std::fprintf(stderr, "[gripper motion] ");

    va_list args;
    va_start(args, format);
    std::vfprintf(stderr, format, args);
    va_end(args);

    std::fprintf(stderr, "\n");
}

inline const char* kCommConfigMayHaveAppliedMessage()
{
    return "communication config may have been applied; reconnect with new address/baudrate";
}
} // namespace

GripperDevice::GripperDevice(const GripperDeviceConfig& config)
    : config_(config),
      motor_(config.device_address),
      last_error_(),
      initialized_(false),
      calibrated_limits_valid_(false),
      open_limit_count_(0),
      safe_open_limit_count_(0),
      close_limit_count_(0),
      safe_close_limit_count_(0)
{
    motor_.setTimeoutMs(config_.timeout_ms);
}

GripperDevice::GripperDevice(const GripperDeviceConfig& config,
                             std::unique_ptr<ITransport> transport)
    : config_(config),
      motor_(config.device_address, std::move(transport)),
      last_error_(),
      initialized_(false),
      calibrated_limits_valid_(false),
      open_limit_count_(0),
      safe_open_limit_count_(0),
      close_limit_count_(0),
      safe_close_limit_count_(0)
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

    invalidateCalibration();
    last_error_.clear();
    return true;
}

void GripperDevice::disconnect()
{
    motor_.disconnect();
    invalidateCalibration();
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
    invalidateCalibration();
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

    if (!(config.current_limit_amp > 0.0f))
    {
        last_error_ = "invalid initialize config: current_limit_amp must be > 0";
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

    if (config.open_safety_margin_mm < 0.0f ||
        config.open_safety_margin_mm >= maxOpeningMm())
    {
        last_error_ = "invalid initialize config: open_safety_margin_mm must be >= 0 and < max opening";
        return false;
    }

    if (config.close_safety_margin_mm < 0.0f ||
        config.close_safety_margin_mm >= maxOpeningMm() ||
        config.open_safety_margin_mm + config.close_safety_margin_mm >= maxOpeningMm())
    {
        last_error_ = "invalid initialize config: close/open safety margins leave no usable travel";
        return false;
    }

    invalidateCalibration();

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

    MotionControlParameters motion_params{};
    if (!motor_.readMotionControlParameters(motion_params))
    {
        setLastErrorFromMotor();
        return false;
    }

    const float current_limit_amp =
        static_cast<float>(motion_params.speed_output_limit) * 0.001f;
    homingDebugLog("current limit before homing speed_output_limit=%lu current_limit_amp=%.3f",
                   static_cast<unsigned long>(motion_params.speed_output_limit),
                   static_cast<double>(current_limit_amp));

    const double current_limit_raw =
        std::round(static_cast<double>(config.current_limit_amp) * 1000.0);
    if (current_limit_raw > static_cast<double>(std::numeric_limits<uint32_t>::max()))
    {
        last_error_ = "invalid initialize config: current_limit_amp is too large";
        return false;
    }

    motion_params.speed_output_limit = static_cast<uint32_t>(current_limit_raw);
    if (!motor_.writeMotionControlParametersTemp(motion_params, nullptr))
    {
        setLastErrorFromMotor();
        return false;
    }

    const float applied_current_limit_amp =
        static_cast<float>(motion_params.speed_output_limit) * 0.001f;
    if (out != nullptr)
    {
        out->current_limit_amp_during_homing = applied_current_limit_amp;
    }
    homingDebugLog("set homing current limit speed_output_limit=%lu current_limit_amp=%.3f",
                   static_cast<unsigned long>(motion_params.speed_output_limit),
                   static_cast<double>(applied_current_limit_amp));

    const float search_speed_rpm =
        openingSpeedMmSToMotorRpmConservative(config.search_speed_mm_s);

    homingDebugLog("start search_direction=%d search_speed_mm_s=%.3f search_speed_rpm=%.3f timeout_ms=%d current_threshold=%.3f backoff_mm=%.3f",
                   config.search_direction,
                   static_cast<double>(config.search_speed_mm_s),
                   static_cast<double>(search_speed_rpm),
                   config.timeout_ms,
                   static_cast<double>(config.current_threshold_a),
                   static_cast<double>(config.backoff_after_zero_mm));

    RealtimeStatus open_limit_before_zero{};
    int open_detect_samples = 0;
    if (!searchLimitByStall(config,
                            config.search_direction,
                            search_speed_rpm,
                            "fault occurred during opening limit search",
                            "opening limit search timeout",
                            open_limit_before_zero,
                            open_detect_samples))
    {
        return false;
    }

    homingDebugStatus("open limit before zero", open_limit_before_zero);

    if (out != nullptr)
    {
        out->limit_detected = true;
        out->detect_samples = open_detect_samples;
        out->limit_opening_mm_before_zero =
            countToOpeningMm(open_limit_before_zero.multi_turn_count);
    }

    const RealtimeStatus open_limit = open_limit_before_zero;
    if (out != nullptr && config.set_zero_after_detect)
    {
        out->zero_set = true;
        out->mechanical_offset = 0;
    }

    if (!releaseOpeningLimit(config, config.search_direction, search_speed_rpm, open_limit))
    {
        return false;
    }

    RealtimeStatus close_limit{};
    int close_detect_samples = 0;
    if (!searchLimitByStall(config,
                            -config.search_direction,
                            search_speed_rpm,
                            "fault occurred during closing limit search",
                            "closing limit search timeout",
                            close_limit,
                            close_detect_samples))
    {
        return false;
    }

    homingDebugStatus("close limit", close_limit);

    open_limit_count_ = open_limit.multi_turn_count;
    close_limit_count_ = close_limit.multi_turn_count;
    calibrated_limits_valid_ = (open_limit_count_ != close_limit_count_);

    if (!calibrated_limits_valid_)
    {
        last_error_ = "invalid homing limits: open and close counts are equal";
        return false;
    }

    const double usable_open_ratio =
        1.0 - static_cast<double>(config.open_safety_margin_mm) /
                  static_cast<double>(maxOpeningMm());
    const double safe_open_count =
        static_cast<double>(close_limit_count_) +
        usable_open_ratio *
            static_cast<double>(open_limit_count_ - close_limit_count_);
    safe_open_limit_count_ = static_cast<int32_t>(std::lround(safe_open_count));
    const double safe_close_ratio =
        static_cast<double>(config.close_safety_margin_mm) /
        static_cast<double>(maxOpeningMm());
    const double safe_close_count =
        static_cast<double>(close_limit_count_) +
        safe_close_ratio *
            static_cast<double>(open_limit_count_ - close_limit_count_);
    safe_close_limit_count_ = static_cast<int32_t>(std::lround(safe_close_count));
    if (safe_open_limit_count_ == safe_close_limit_count_)
    {
        calibrated_limits_valid_ = false;
        last_error_ = "invalid homing limits: open safety margin leaves no usable travel";
        return false;
    }

    const double center_count_value =
        (static_cast<double>(safe_open_limit_count_) +
         static_cast<double>(safe_close_limit_count_)) /
        2.0;
    const int32_t center_count = static_cast<int32_t>(std::lround(center_count_value));

    homingDebugLog("center target mechanical_open_count=%ld safe_open_count=%ld mechanical_close_count=%ld safe_close_count=%ld open_safety_margin_mm=%.3f close_safety_margin_mm=%.3f center_count=%ld",
                   static_cast<long>(open_limit.multi_turn_count),
                   static_cast<long>(safe_open_limit_count_),
                   static_cast<long>(close_limit.multi_turn_count),
                   static_cast<long>(safe_close_limit_count_),
                   static_cast<double>(config.open_safety_margin_mm),
                   static_cast<double>(config.close_safety_margin_mm),
                   static_cast<long>(center_count));

    RealtimeStatus latest{};
    if (!motor_.moveToCount(center_count, &latest))
    {
        setLastErrorFromMotor();
        return false;
    }

    homingDebugStatus("center move response", latest);

    if (!waitForTargetCount(config, center_count, latest))
    {
        return false;
    }

    homingDebugStatus("center final", latest);

    if (out != nullptr)
    {
        out->detect_samples = open_detect_samples + close_detect_samples;
        convertRealtimeToStatus(latest, out->final_status);
    }

    initialized_ = true;
    last_error_.clear();
    return true;
}

bool GripperDevice::searchLimitByStall(const GripperInitializeConfig& config,
                                       int search_direction,
                                       float search_speed_rpm,
                                       const char* fault_context,
                                       const char* timeout_error,
                                       RealtimeStatus& out_limit_status,
                                       int& out_detect_samples)
{
    const float cmd_speed_rpm =
        static_cast<float>(search_direction) * search_speed_rpm;

    homingDebugLog("search limit start direction=%d cmd_speed_rpm=%.3f",
                   search_direction,
                   static_cast<double>(cmd_speed_rpm));

    if (!motor_.setSpeed(cmd_speed_rpm, 0, nullptr))
    {
        setLastErrorFromMotor();
        return false;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(config.timeout_ms);

    bool has_prev_status = false;
    bool has_start_status = false;
    int32_t start_count = 0;
    RealtimeStatus prev_status{};
    int consecutive_hits = 0;
    int sample_count = 0;
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

        if (!has_start_status)
        {
            start_count = latest.multi_turn_count;
            has_start_status = true;
        }

        if (latest.fault_code != 0)
        {
            motor_.motorOff(nullptr);
            last_error_ = makeFaultContextError(fault_context, latest.fault_code);
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
        const int32_t min_travel_count =
            openingMmToBackoffDeltaCount(1.0f, start_count);
        const std::int64_t moved_count_delta =
            static_cast<std::int64_t>(latest.multi_turn_count) -
            static_cast<std::int64_t>(start_count);
        const std::int64_t moved_count =
            (moved_count_delta < 0) ? -moved_count_delta : moved_count_delta;
        const bool moved_far_enough =
            moved_count >= min_travel_count;
        const bool stalled_at_existing_limit =
            current_high && (speed_small || position_locked);
        const bool stalled_after_travel =
            moved_far_enough && position_locked &&
            (speed_small || current_high);

        if (stalled_at_existing_limit || stalled_after_travel)
        {
            ++consecutive_hits;
        }
        else
        {
            consecutive_hits = 0;
        }

        ++sample_count;
        if ((sample_count <= 5) ||
            (sample_count % 10 == 0) ||
            stalled_at_existing_limit ||
            stalled_after_travel)
        {
            homingDebugLog("search sample direction=%d sample=%d count=%ld delta_mm=%.4f opening_speed=%.4f q_current=%.3f moved_count=%ld speed_small=%d current_high=%d position_locked=%d hits=%d",
                           search_direction,
                           sample_count,
                           static_cast<long>(latest.multi_turn_count),
                           static_cast<double>(delta_opening_mm),
                           static_cast<double>(opening_speed_mm_s),
                           static_cast<double>(latest.q_current_amp),
                           static_cast<long>(moved_count_delta),
                           speed_small ? 1 : 0,
                           current_high ? 1 : 0,
                           position_locked ? 1 : 0,
                           consecutive_hits);
        }

        prev_status = latest;
        has_prev_status = true;

        if (consecutive_hits >= config.detect_consecutive_samples)
        {
            out_limit_status = latest;
            out_detect_samples = consecutive_hits;
            homingDebugStatus("search limit detected", latest);
            return true;
        }
    }

    motor_.motorOff(nullptr);
    last_error_ = timeout_error;
    return false;
}

bool GripperDevice::releaseOpeningLimit(const GripperInitializeConfig& config,
                                        int search_direction,
                                        float search_speed_rpm,
                                        const RealtimeStatus& open_limit)
{
    const float release_mm = std::max(config.backoff_after_zero_mm, 1.0f);
    const int32_t release_count =
        openingMmToBackoffDeltaCount(release_mm, open_limit.multi_turn_count);
    const int32_t min_started_delta = std::min<int32_t>(release_count, 50);
    const float release_speed_rpm =
        -static_cast<float>(search_direction) * search_speed_rpm;

    homingDebugLog("release opening limit release_mm=%.3f open_count=%ld release_count=%ld release_speed_rpm=%.3f min_started_delta=%ld",
                   static_cast<double>(release_mm),
                   static_cast<long>(open_limit.multi_turn_count),
                   static_cast<long>(release_count),
                   static_cast<double>(release_speed_rpm),
                   static_cast<long>(min_started_delta));

    RealtimeStatus latest{};
    if (!motor_.setSpeed(release_speed_rpm, 0, &latest))
    {
        setLastErrorFromMotor();
        return false;
    }

    homingDebugStatus("release speed response", latest);

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(config.timeout_ms);

    int sample_count = 0;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (latest.fault_code != 0)
        {
            motor_.motorOff(nullptr);
            last_error_ = makeFaultContextError("fault occurred while releasing opening limit",
                                                latest.fault_code);
            return false;
        }

        const std::int64_t moved_count_delta =
            static_cast<std::int64_t>(latest.multi_turn_count) -
            static_cast<std::int64_t>(open_limit.multi_turn_count);
        const std::int64_t moved_count =
            (moved_count_delta < 0) ? -moved_count_delta : moved_count_delta;
        ++sample_count;
        if (sample_count <= 5 || sample_count % 10 == 0)
        {
            homingDebugLog("release sample=%d count=%ld moved_delta=%ld moved_count=%ld speed_rpm=%.3f q_current=%.3f fault=0x%02X run_state=%u enabled=%d",
                           sample_count,
                           static_cast<long>(latest.multi_turn_count),
                           static_cast<long>(moved_count_delta),
                           static_cast<long>(moved_count),
                           static_cast<double>(latest.speed_rpm),
                           static_cast<double>(latest.q_current_amp),
                           static_cast<unsigned>(latest.fault_code),
                           static_cast<unsigned>(latest.run_state),
                           latest.motor_enabled ? 1 : 0);
        }

        if (moved_count >= min_started_delta)
        {
            homingDebugStatus("release detected movement", latest);
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config.poll_interval_ms));

        if (!motor_.readRealtime(latest))
        {
            setLastErrorFromMotor();
            motor_.motorOff(nullptr);
            return false;
        }
    }

    motor_.motorOff(nullptr);
    last_error_ = "opening limit release timeout";
    return false;
}

bool GripperDevice::waitForTargetCount(const GripperInitializeConfig& config,
                                       int32_t target_count,
                                       RealtimeStatus& out_status)
{
    const int32_t tolerance_count =
        std::max<int32_t>(10,
                          openingMmToBackoffDeltaCount(
                              std::max(config.position_epsilon_mm, 0.01f),
                              target_count));

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(config.timeout_ms);

    while (true)
    {
        if (out_status.fault_code != 0)
        {
            motor_.motorOff(nullptr);
            last_error_ = makeFaultContextError("fault occurred during centering",
                                                out_status.fault_code);
            return false;
        }

        const float opening_speed_mm_s =
            motorRpmToOpeningSpeedMmS(out_status.speed_rpm, out_status.multi_turn_count);
        const std::int64_t count_delta =
            static_cast<std::int64_t>(out_status.multi_turn_count) -
            static_cast<std::int64_t>(target_count);
        const std::int64_t count_error =
            (count_delta < 0) ? -count_delta : count_delta;

        if (count_error <= tolerance_count &&
            std::abs(opening_speed_mm_s) <= config.speed_epsilon_mm_s)
        {
            return true;
        }

        if (std::chrono::steady_clock::now() >= deadline)
        {
            motor_.motorOff(nullptr);
            last_error_ = "center move timeout";
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config.poll_interval_ms));

        if (!motor_.readRealtime(out_status))
        {
            setLastErrorFromMotor();
            motor_.motorOff(nullptr);
            return false;
        }
    }
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

bool GripperDevice::moveToOpeningMmSmooth(float target_opening_mm,
                                          const PositionMotionConfig& config,
                                          GripperStatus* out)
{
    if (!initialized_)
    {
        last_error_ = "gripper not initialized";
        return false;
    }

    if (!(config.max_speed_mm_s > 0.0f) ||
        !(config.acceleration_mm_s2 > 0.0f) ||
        !(config.max_current_amp > 0.0f) ||
        config.braking_margin_mm < 0.0f ||
        !(config.final_position_speed_mm_s > 0.0f) ||
        config.position_tolerance_mm < 0.0f ||
        config.speed_epsilon_mm_s < 0.0f ||
        config.update_interval_ms <= 0 ||
        config.timeout_ms <= 0)
    {
        last_error_ = "invalid smooth move config";
        return false;
    }

    int32_t target_count = 0;
    if (!openingMmToCount(target_opening_mm, target_count))
    {
        return false;
    }

    RealtimeStatus latest{};
    if (!motor_.readRealtime(latest))
    {
        setLastErrorFromMotor();
        return false;
    }
    if (latest.fault_code != 0)
    {
        last_error_ = makeFaultContextError("fault occurred before smooth position move",
                                            latest.fault_code);
        return false;
    }

    const double calibrated_span_count =
        std::abs(static_cast<double>(safe_open_limit_count_) -
                 static_cast<double>(safe_close_limit_count_));
    const double counts_per_mm =
        calibrated_span_count / static_cast<double>(maxOpeningMm() - minOpeningMm());
    const double max_speed_rpm =
        static_cast<double>(config.max_speed_mm_s) * counts_per_mm * 60.0 /
        kEncoderCountPerRev;
    const double acceleration_rpm_s =
        static_cast<double>(config.acceleration_mm_s2) * counts_per_mm * 60.0 /
        kEncoderCountPerRev;
    const double acceleration_raw_value = std::round(acceleration_rpm_s * 100.0);
    if (acceleration_raw_value < 1.0 ||
        acceleration_raw_value > static_cast<double>(std::numeric_limits<uint32_t>::max()))
    {
        last_error_ = "invalid smooth move config";
        return false;
    }
    const uint32_t acceleration_raw =
        static_cast<uint32_t>(acceleration_raw_value);

    MotionControlParameters motion_params{};
    if (!motor_.readMotionControlParameters(motion_params))
    {
        setLastErrorFromMotor();
        return false;
    }
    const double current_limit_raw_value =
        std::round(static_cast<double>(config.max_current_amp) * 1000.0);
    if (current_limit_raw_value < 1.0 ||
        current_limit_raw_value > static_cast<double>(std::numeric_limits<uint32_t>::max()))
    {
        last_error_ = "invalid smooth move config";
        return false;
    }
    motion_params.speed_output_limit =
        static_cast<uint32_t>(current_limit_raw_value);
    if (!motor_.writeMotionControlParametersTemp(motion_params, nullptr))
    {
        setLastErrorFromMotor();
        return false;
    }

    const double tolerance_count =
        std::max(10.0,
                 static_cast<double>(config.position_tolerance_mm) * counts_per_mm);
    const double braking_margin_count =
        static_cast<double>(config.braking_margin_mm) * counts_per_mm;
    const double acceleration_count_s2 =
        static_cast<double>(config.acceleration_mm_s2) * counts_per_mm;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(config.timeout_ms);

    const double initial_error_count =
        static_cast<double>(target_count) - static_cast<double>(latest.multi_turn_count);
    const int direction = (initial_error_count >= 0.0) ? 1 : -1;

    motionDebugLog("speed-profile move start actual_count=%ld target_count=%ld max_speed_rpm=%.3f acceleration_rpm_s=%.3f braking_margin_mm=%.3f",
                   static_cast<long>(latest.multi_turn_count),
                   static_cast<long>(target_count),
                   max_speed_rpm,
                   acceleration_rpm_s,
                   static_cast<double>(config.braking_margin_mm));

    if (std::abs(initial_error_count) > tolerance_count)
    {
        if (!motor_.setSpeed(static_cast<float>(direction * max_speed_rpm),
                             acceleration_raw,
                             &latest))
        {
            setLastErrorFromMotor();
            return false;
        }

        int sample_count = 0;
        while (true)
        {
            if (latest.fault_code != 0)
            {
                motor_.setSpeed(0.0f, acceleration_raw, nullptr);
                last_error_ = makeFaultContextError("fault occurred during smooth speed move",
                                                    latest.fault_code);
                return false;
            }

            const double remaining_count =
                static_cast<double>(direction) *
                (static_cast<double>(target_count) -
                 static_cast<double>(latest.multi_turn_count));
            const double actual_speed_count_s =
                std::abs(static_cast<double>(latest.speed_rpm)) *
                kEncoderCountPerRev / 60.0;
            const double braking_distance_count =
                (actual_speed_count_s * actual_speed_count_s) /
                (2.0 * acceleration_count_s2);

            ++sample_count;
            if (sample_count <= 5 || sample_count % 10 == 0)
            {
                motionDebugLog("speed sample=%d actual_count=%ld target_count=%ld remaining_mm=%.3f speed_mm_s=%.3f braking_mm=%.3f q_current=%.3f",
                               sample_count,
                               static_cast<long>(latest.multi_turn_count),
                               static_cast<long>(target_count),
                               remaining_count / counts_per_mm,
                               actual_speed_count_s / counts_per_mm,
                               braking_distance_count / counts_per_mm,
                               static_cast<double>(latest.q_current_amp));
            }

            if (remaining_count <= braking_distance_count + braking_margin_count)
            {
                break;
            }

            if (std::chrono::steady_clock::now() >= deadline)
            {
                motor_.setSpeed(0.0f, acceleration_raw, nullptr);
                last_error_ = "smooth position move timeout";
                return false;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(config.update_interval_ms));
            if (!motor_.readRealtime(latest))
            {
                setLastErrorFromMotor();
                motor_.setSpeed(0.0f, acceleration_raw, nullptr);
                return false;
            }
        }

        if (!motor_.setSpeed(0.0f, acceleration_raw, &latest))
        {
            setLastErrorFromMotor();
            return false;
        }

        while (true)
        {
            const double actual_speed_mm_s =
                std::abs(static_cast<double>(latest.speed_rpm)) *
                kEncoderCountPerRev / 60.0 / counts_per_mm;
            if (actual_speed_mm_s <= config.speed_epsilon_mm_s)
            {
                break;
            }

            if (std::chrono::steady_clock::now() >= deadline)
            {
                last_error_ = "smooth position move timeout";
                return false;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(config.update_interval_ms));
            if (!motor_.readRealtime(latest))
            {
                setLastErrorFromMotor();
                return false;
            }
            if (latest.fault_code != 0)
            {
                last_error_ = makeFaultContextError("fault occurred while braking smooth move",
                                                    latest.fault_code);
                return false;
            }
        }
    }

    const float final_speed_mm_s =
        std::min(config.final_position_speed_mm_s, config.max_speed_mm_s);
    const float final_speed_rpm =
        openingSpeedMmSToMotorRpm(final_speed_mm_s, latest.multi_turn_count);
    if (!motor_.moveToCountWithLimits(target_count,
                                      final_speed_rpm,
                                      config.max_current_amp,
                                      &latest))
    {
        setLastErrorFromMotor();
        return false;
    }

    while (true)
    {
        if (latest.fault_code != 0)
        {
            last_error_ = makeFaultContextError("fault occurred during final position hold",
                                                latest.fault_code);
            return false;
        }

        const double count_error =
            std::abs(static_cast<double>(latest.multi_turn_count) -
                     static_cast<double>(target_count));
        const double actual_speed_mm_s =
            std::abs(static_cast<double>(latest.speed_rpm)) *
            kEncoderCountPerRev / 60.0 / counts_per_mm;
        if (count_error <= tolerance_count &&
            actual_speed_mm_s <= config.speed_epsilon_mm_s)
        {
            if (out != nullptr)
            {
                convertRealtimeToStatus(latest, *out);
            }
            last_error_.clear();
            return true;
        }

        if (std::chrono::steady_clock::now() >= deadline)
        {
            motionDebugLog("final hold timeout actual_count=%ld target_count=%ld error_mm=%.3f speed_mm_s=%.3f q_current=%.3f",
                           static_cast<long>(latest.multi_turn_count),
                           static_cast<long>(target_count),
                           count_error / counts_per_mm,
                           actual_speed_mm_s,
                           static_cast<double>(latest.q_current_amp));
            last_error_ = "smooth position move timeout";
            return false;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(config.update_interval_ms));
        if (!motor_.readRealtime(latest))
        {
            setLastErrorFromMotor();
            return false;
        }
    }
}

bool GripperDevice::moveToOpeningMmPositionProfile(float target_opening_mm,
                                                   const PositionMotionConfig& config,
                                                   GripperStatus* out)
{
    if (!initialized_)
    {
        last_error_ = "gripper not initialized";
        return false;
    }

    if (!(config.max_speed_mm_s > 0.0f) ||
        !(config.acceleration_mm_s2 > 0.0f) ||
        !(config.max_current_amp > 0.0f) ||
        !(config.max_following_error_mm > 0.0f) ||
        config.position_tolerance_mm < 0.0f ||
        config.speed_epsilon_mm_s < 0.0f ||
        config.update_interval_ms <= 0 ||
        config.timeout_ms <= 0)
    {
        last_error_ = "invalid smooth move config";
        return false;
    }

    int32_t target_count = 0;
    if (!openingMmToCount(target_opening_mm, target_count))
    {
        return false;
    }

    RealtimeStatus current{};
    if (!motor_.readRealtime(current))
    {
        setLastErrorFromMotor();
        return false;
    }
    if (current.fault_code != 0)
    {
        last_error_ = makeFaultContextError("fault occurred before smooth position move",
                                            current.fault_code);
        return false;
    }

    const float max_speed_rpm =
        openingSpeedMmSToMotorRpm(config.max_speed_mm_s, current.multi_turn_count);

    // Enter position mode at the measured position first.  This avoids applying a
    // large position error at the instant the controller changes mode.
    RealtimeStatus latest{};
    if (!motor_.moveToCountWithLimits(current.multi_turn_count,
                                      max_speed_rpm,
                                      config.max_current_amp,
                                      &latest))
    {
        setLastErrorFromMotor();
        return false;
    }

    const double calibrated_span_count =
        std::abs(static_cast<double>(safe_open_limit_count_) -
                 static_cast<double>(safe_close_limit_count_));
    const double counts_per_mm =
        calibrated_span_count / static_cast<double>(maxOpeningMm() - minOpeningMm());
    const double max_speed_count_s =
        static_cast<double>(config.max_speed_mm_s) * counts_per_mm;
    const double acceleration_count_s2 =
        static_cast<double>(config.acceleration_mm_s2) * counts_per_mm;
    const double max_following_error_count =
        static_cast<double>(config.max_following_error_mm) * counts_per_mm;
    const double position_tolerance_count =
        std::max(10.0,
                 static_cast<double>(config.position_tolerance_mm) * counts_per_mm);

    motionDebugLog("smooth move start actual_count=%ld target_count=%ld max_speed_mm_s=%.3f acceleration_mm_s2=%.3f following_error_mm=%.3f interval_ms=%d timeout_ms=%d",
                   static_cast<long>(current.multi_turn_count),
                   static_cast<long>(target_count),
                   static_cast<double>(config.max_speed_mm_s),
                   static_cast<double>(config.acceleration_mm_s2),
                   static_cast<double>(config.max_following_error_mm),
                   config.update_interval_ms,
                   config.timeout_ms);

    const int direction =
        (target_count >= current.multi_turn_count) ? 1 : -1;
    double commanded_count = static_cast<double>(current.multi_turn_count);
    double profile_speed_count_s = 0.0;

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(config.timeout_ms);
    auto last_profile_time = std::chrono::steady_clock::now();
    auto next_update =
        last_profile_time + std::chrono::milliseconds(config.update_interval_ms);
    int sample_count = 0;

    while (std::abs(static_cast<double>(target_count) - commanded_count) >= 0.5)
    {
        if (std::chrono::steady_clock::now() >= deadline)
        {
            last_error_ = "smooth position move timeout";
            return false;
        }

        std::this_thread::sleep_until(next_update);
        const auto now = std::chrono::steady_clock::now();
        const double dt_s =
            std::max(1e-4,
                     std::chrono::duration<double>(now - last_profile_time).count());
        last_profile_time = now;
        next_update += std::chrono::milliseconds(config.update_interval_ms);
        if (next_update < now)
        {
            next_update = now;
        }

        const double remaining_count =
            std::abs(static_cast<double>(target_count) - commanded_count);
        const double braking_speed_count_s =
            std::sqrt(2.0 * acceleration_count_s2 * remaining_count);
        const double desired_speed_count_s =
            std::min(max_speed_count_s, braking_speed_count_s);
        const double speed_step_count_s = acceleration_count_s2 * dt_s;
        if (profile_speed_count_s < desired_speed_count_s)
        {
            profile_speed_count_s =
                std::min(desired_speed_count_s,
                         profile_speed_count_s + speed_step_count_s);
        }
        else
        {
            profile_speed_count_s =
                std::max(desired_speed_count_s,
                         profile_speed_count_s - speed_step_count_s);
        }

        const double profile_step_count =
            std::min(remaining_count, profile_speed_count_s * dt_s);
        double next_command_count =
            commanded_count + static_cast<double>(direction) * profile_step_count;
        const double profile_command_count = next_command_count;

        // Do not let the host-generated target run too far ahead of the motor.
        const double following_limit_count =
            static_cast<double>(latest.multi_turn_count) +
            static_cast<double>(direction) * max_following_error_count;
        if (direction > 0)
        {
            next_command_count =
                std::min(next_command_count,
                         std::min(static_cast<double>(target_count), following_limit_count));
            next_command_count = std::max(next_command_count, commanded_count);
        }
        else
        {
            next_command_count =
                std::max(next_command_count,
                         std::max(static_cast<double>(target_count), following_limit_count));
            next_command_count = std::min(next_command_count, commanded_count);
        }
        const bool following_limited =
            std::abs(next_command_count - profile_command_count) >= 0.5;

        int32_t next_count = static_cast<int32_t>(std::lround(next_command_count));
        const int32_t previous_command_count =
            static_cast<int32_t>(std::lround(commanded_count));
        if (next_count == previous_command_count)
        {
            if (!motor_.readRealtime(latest))
            {
                setLastErrorFromMotor();
                return false;
            }
        }
        else
        {
            if (!motor_.moveToCount(next_count, &latest))
            {
                setLastErrorFromMotor();
                return false;
            }
            commanded_count = static_cast<double>(next_count);
        }

        if (latest.fault_code != 0)
        {
            last_error_ = makeFaultContextError("fault occurred during smooth position move",
                                                latest.fault_code);
            return false;
        }

        ++sample_count;
        if (sample_count <= 5 || sample_count % 10 == 0 || following_limited)
        {
            const double following_error_mm =
                std::abs(commanded_count - static_cast<double>(latest.multi_turn_count)) /
                counts_per_mm;
            motionDebugLog("sample=%d command_count=%ld actual_count=%ld target_count=%ld profile_speed_mm_s=%.3f actual_speed_mm_s=%.3f q_current=%.3f following_error_mm=%.3f following_limited=%d",
                           sample_count,
                           static_cast<long>(std::lround(commanded_count)),
                           static_cast<long>(latest.multi_turn_count),
                           static_cast<long>(target_count),
                           profile_speed_count_s / counts_per_mm,
                           static_cast<double>(motorRpmToOpeningSpeedMmS(
                               latest.speed_rpm, latest.multi_turn_count)),
                           static_cast<double>(latest.q_current_amp),
                           following_error_mm,
                           following_limited ? 1 : 0);
        }
    }

    if (static_cast<int32_t>(std::lround(commanded_count)) != target_count)
    {
        if (!motor_.moveToCount(target_count, &latest))
        {
            setLastErrorFromMotor();
            return false;
        }
    }

    while (true)
    {
        if (latest.fault_code != 0)
        {
            last_error_ = makeFaultContextError("fault occurred while settling smooth position move",
                                                latest.fault_code);
            return false;
        }

        const double count_error =
            std::abs(static_cast<double>(latest.multi_turn_count) -
                     static_cast<double>(target_count));
        const float opening_speed_mm_s =
            motorRpmToOpeningSpeedMmS(latest.speed_rpm, latest.multi_turn_count);
        if (count_error <= position_tolerance_count &&
            std::abs(opening_speed_mm_s) <= config.speed_epsilon_mm_s)
        {
            if (out != nullptr)
            {
                convertRealtimeToStatus(latest, *out);
            }
            last_error_.clear();
            return true;
        }

        if (std::chrono::steady_clock::now() >= deadline)
        {
            last_error_ = "smooth position move timeout";
            motionDebugLog("smooth move timeout while settling actual_count=%ld target_count=%ld speed_mm_s=%.3f q_current=%.3f",
                           static_cast<long>(latest.multi_turn_count),
                           static_cast<long>(target_count),
                           static_cast<double>(opening_speed_mm_s),
                           static_cast<double>(latest.q_current_amp));
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config.update_interval_ms));
        if (!motor_.readRealtime(latest))
        {
            setLastErrorFromMotor();
            return false;
        }
    }
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

    invalidateCalibration();
    last_error_.clear();
    return true;
}

bool GripperDevice::setCommunicationConfig(uint8_t new_device_address, int new_baudrate)
{
    if (new_device_address == 0 || new_device_address == 0xFF)
    {
        last_error_ = "invalid device address: expected 1~254";
        return false;
    }

    Rs485BaudrateCode baudrate_code = Rs485BaudrateCode::Baud115200;
    if (!baudrateToCode(new_baudrate, baudrate_code))
    {
        last_error_ = "invalid RS485 baudrate: expected one of 9600/19200/38400/57600/115200/460800/921600";
        return false;
    }

    UserParameters current{};
    if (!motor_.readUserParameters(current))
    {
        setLastErrorFromMotor();
        return false;
    }

    const bool address_changed = (current.device_address != new_device_address);
    const bool baudrate_changed = (current.rs485_baudrate != baudrate_code);
    const bool config_changed = address_changed || baudrate_changed;

    if (!config_changed)
    {
        last_error_.clear();
        return true;
    }

    WritableUserParameters writable{};
    writable.encoder_model = current.encoder_model;
    writable.invert_encoder_direction = current.invert_encoder_direction;
    writable.enable_second_encoder = current.enable_second_encoder;
    writable.speed_filter_coeff = current.speed_filter_coeff;
    writable.device_address = new_device_address;
    writable.rs485_baudrate = baudrate_code;
    writable.can_baudrate = current.can_baudrate;
    writable.enable_canopen = current.enable_canopen;
    writable.max_bus_voltage_0p01v = current.max_bus_voltage_0p01v;
    writable.voltage_fault_delay_s = current.voltage_fault_delay_s;
    writable.max_bus_current_0p01a = current.max_bus_current_0p01a;
    writable.current_fault_delay_s = current.current_fault_delay_s;
    writable.max_temperature_c = current.max_temperature_c;
    writable.temperature_fault_delay_s = current.temperature_fault_delay_s;

    if (!motor_.writeUserParameters(writable, nullptr))
    {
        setLastErrorFromMotor();

        if (config_changed && isReadTimeoutError(last_error_))
        {
            config_.device_address = new_device_address;
            config_.baudrate = new_baudrate;
            motor_.setDeviceAddress(new_device_address);
            invalidateCalibration();
            last_error_ = kCommConfigMayHaveAppliedMessage();
        }

        return false;
    }

    config_.device_address = new_device_address;
    config_.baudrate = new_baudrate;
    motor_.setDeviceAddress(new_device_address);
    invalidateCalibration();
    last_error_.clear();
    return true;
}

void GripperDevice::invalidateCalibration()
{
    initialized_ = false;
    calibrated_limits_valid_ = false;
    open_limit_count_ = 0;
    safe_open_limit_count_ = 0;
    close_limit_count_ = 0;
    safe_close_limit_count_ = 0;
}

float GripperDevice::minOpeningMm() const
{
    return 0.0f;
}

float GripperDevice::maxOpeningMm() const
{
    return static_cast<float>(kSoftwareMaxOpeningMm);
}

double GripperDevice::countToTurbineAngleDeg(int32_t count) const
{
    return -static_cast<double>(count) * kDegPerCount;
}

double GripperDevice::turbineAngleDegToOpeningMm(double alpha_deg) const
{
    const double alpha = std::clamp(alpha_deg, kAlphaMinDeg, kAlphaMaxDeg);

    const double formula_opening =
        2.0 * kLinkLengthMm *
        (std::sin(degToRad(kAlphaBreakDeg - alpha)) +
         std::sin(degToRad(kAlphaOffsetDeg)));
    return formula_opening * openingFormulaScale();
}

float GripperDevice::countToOpeningMm(int32_t count) const
{
    if (calibrated_limits_valid_)
    {
        const double opening_ratio =
            (static_cast<double>(count) - static_cast<double>(safe_close_limit_count_)) /
            (static_cast<double>(safe_open_limit_count_) - static_cast<double>(safe_close_limit_count_));
        const double opening =
            std::clamp(opening_ratio, 0.0, 1.0) *
            static_cast<double>(maxOpeningMm());
        return static_cast<float>(opening);
    }

    const double opening = turbineAngleDegToOpeningMm(countToTurbineAngleDeg(count));
    return static_cast<float>(std::clamp(opening,
                                         static_cast<double>(minOpeningMm()),
                                         static_cast<double>(maxOpeningMm())));
}

double GripperDevice::openingSpeedScaleMmPerSecPerRpm(double alpha_deg) const
{
    const double alpha = std::clamp(alpha_deg, kAlphaMinDeg, kAlphaMaxDeg);
    return (kPi / 6.0) *
           std::cos(degToRad(kAlphaBreakDeg - alpha)) *
           openingFormulaScale();
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

    const double max_scale = (kPi / 6.0) * openingFormulaScale();
    return static_cast<float>(static_cast<double>(opening_speed_mm_s) / max_scale);
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

    if (calibrated_limits_valid_)
    {
        const double ratio = (target_mm - min_mm) / (max_mm - min_mm);
        const double target_count =
            static_cast<double>(safe_close_limit_count_) +
            ratio * static_cast<double>(safe_open_limit_count_ - safe_close_limit_count_);
        out_count = static_cast<int32_t>(std::lround(target_count));
        last_error_.clear();
        return true;
    }

    const double formula_target_mm = target_mm / openingFormulaScale();
    const double s = formula_target_mm / (2.0 * kLinkLengthMm) -
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
