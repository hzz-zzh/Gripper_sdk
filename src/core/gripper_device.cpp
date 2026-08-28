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
      close_limit_count_(0)
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
      close_limit_count_(0)
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

    const std::int64_t center_count_sum =
        static_cast<std::int64_t>(safe_open_limit_count_) +
        static_cast<std::int64_t>(close_limit_count_);
    const int32_t center_count = static_cast<int32_t>(center_count_sum / 2);

    homingDebugLog("center target mechanical_open_count=%ld safe_open_count=%ld close_count=%ld open_safety_margin_mm=%.3f center_count=%ld",
                   static_cast<long>(open_limit.multi_turn_count),
                   static_cast<long>(safe_open_limit_count_),
                   static_cast<long>(close_limit.multi_turn_count),
                   static_cast<double>(config.open_safety_margin_mm),
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
            (static_cast<double>(count) - static_cast<double>(close_limit_count_)) /
            (static_cast<double>(safe_open_limit_count_) - static_cast<double>(close_limit_count_));
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
            static_cast<double>(close_limit_count_) +
            ratio * static_cast<double>(safe_open_limit_count_ - close_limit_count_);
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
