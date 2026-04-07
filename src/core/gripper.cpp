#include "core/gripper.h"
#include "transport/rs485_transport.h"

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
uint32_t convertSpeedLimitRpmToRaw(float rpm, bool& ok)
{
    ok = false;
    if (!(rpm > 0.0f))
    {
        return 0;
    }

    const float raw = std::round(rpm * 100.0f);
    if (raw < 0.0f || raw > static_cast<float>(std::numeric_limits<uint32_t>::max()))
    {
        return 0;
    }

    ok = true;
    return static_cast<uint32_t>(raw);
}

uint32_t convertCurrentLimitAmpToRaw(float current_amp, bool& ok)
{
    ok = false;
    if (!(current_amp > 0.0f))
    {
        return 0;
    }

    const float raw = std::round(current_amp * 1000.0f);
    if (raw < 0.0f || raw > static_cast<float>(std::numeric_limits<uint32_t>::max()))
    {
        return 0;
    }

    ok = true;
    return static_cast<uint32_t>(raw);
}

int remainingMs(const std::chrono::steady_clock::time_point& deadline)
{
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline)
    {
        return 0;
    }

    return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
}

bool readExactUntil(ITransport& transport,
                    uint8_t* buffer,
                    std::size_t size,
                    const std::chrono::steady_clock::time_point& deadline,
                    std::string& error)
{
    std::size_t received = 0;

    while (received < size)
    {
        const int remain_ms = remainingMs(deadline);
        if (remain_ms <= 0)
        {
            error = "read timeout";
            return false;
        }

        const int ret = transport.readBytes(buffer + received, size - received, remain_ms);
        if (ret < 0)
        {
            error = "transport read failed";
            return false;
        }
        if (ret == 0)
        {
            continue;
        }

        received += static_cast<std::size_t>(ret);
    }

    return true;
}
} // namespace

Gripper::Gripper(uint8_t device_address)
    : transport_(nullptr),
      device_address_(device_address),
      next_sequence_(0),
      timeout_ms_(200),
      last_error_()
{
}

Gripper::Gripper(uint8_t device_address, std::unique_ptr<ITransport> transport)
    : transport_(std::move(transport)),
      device_address_(device_address),
      next_sequence_(0),
      timeout_ms_(200),
      last_error_()
{
}

bool Gripper::connect(const std::string& port_name, int baudrate)
{
    if (transport_)
    {
        if (!transport_->isOpen() && !transport_->open())
        {
            last_error_ = "failed to open transport";
            return false;
        }

        last_error_.clear();
        return true;
    }

    transport_ = std::make_unique<Rs485Transport>(port_name, baudrate);
    if (!transport_->open())
    {
        last_error_ = "failed to open rs485 port";
        transport_.reset();
        return false;
    }

    last_error_.clear();
    return true;
}

void Gripper::disconnect()
{
    if (transport_)
    {
        transport_->close();
        transport_.reset();
    }
}

bool Gripper::isConnected() const
{
    return transport_ && transport_->isOpen();
}

void Gripper::setTimeoutMs(int timeout_ms)
{
    timeout_ms_ = timeout_ms;
}

const std::string& Gripper::lastError() const
{
    return last_error_;
}

bool Gripper::readVersion(VersionInfo& out)
{
    protocol::Frame response;
    if (!transact(protocol::Command::ReadVersion, {}, response, true))
    {
        return false;
    }

    if (response.payload.size() != 0x16)
    {
        last_error_ = "invalid version payload length";
        return false;
    }

    out.boot_version = protocol::readU16LE(&response.payload[0]);
    out.app_version = protocol::readU16LE(&response.payload[2]);
    out.hardware_model = protocol::readU16LE(&response.payload[4]);
    out.rs485_custom_version = response.payload[6];
    out.rs485_modbus_version = response.payload[7];
    out.can_custom_version = response.payload[8];
    out.canopen_version = response.payload[9];

    for (std::size_t i = 0; i < out.uid.size(); ++i)
    {
        out.uid[i] = response.payload[10 + i];
    }

    return true;
}

bool Gripper::readRealtime(RealtimeStatus& out)
{
    protocol::Frame response;
    if (!transact(protocol::Command::ReadRealtime, {}, response, true))
    {
        return false;
    }

    return parseRealtimePayload(response.payload, out, last_error_);
}

bool Gripper::clearFault(uint8_t& current_fault)
{
    protocol::Frame response;
    if (!transact(protocol::Command::ClearFault, {}, response, true))
    {
        return false;
    }

    if (response.payload.size() != 1)
    {
        last_error_ = "invalid clear-fault payload length";
        return false;
    }

    current_fault = response.payload[0];
    return true;
}

bool Gripper::setQCurrent(float current_amp,
                          uint32_t slope_milliamp_per_sec,
                          RealtimeStatus* out)
{
    std::vector<uint8_t> payload;
    const int32_t target_milliamp = static_cast<int32_t>(current_amp * 1000.0f);

    protocol::appendI32LE(payload, target_milliamp);
    protocol::appendU32LE(payload, slope_milliamp_per_sec);

    protocol::Frame response;
    if (!transact(protocol::Command::WriteQCurrent, payload, response, true))
    {
        return false;
    }

    if (out)
    {
        return parseRealtimePayload(response.payload, *out, last_error_);
    }

    return true;
}

bool Gripper::setSpeed(float rpm,
                       uint32_t accel_0p01rpm_per_sec,
                       RealtimeStatus* out)
{
    std::vector<uint8_t> payload;
    const int32_t target_speed = static_cast<int32_t>(rpm * 100.0f);

    protocol::appendI32LE(payload, target_speed);
    protocol::appendU32LE(payload, accel_0p01rpm_per_sec);

    protocol::Frame response;
    if (!transact(protocol::Command::WriteSpeed, payload, response, true))
    {
        return false;
    }

    if (out)
    {
        return parseRealtimePayload(response.payload, *out, last_error_);
    }

    return true;
}

bool Gripper::moveToCount(int32_t target_count, RealtimeStatus* out)
{
    std::vector<uint8_t> payload;
    protocol::appendI32LE(payload, target_count);

    protocol::Frame response;
    if (!transact(protocol::Command::MoveAbsolute, payload, response, true))
    {
        return false;
    }

    if (out)
    {
        return parseRealtimePayload(response.payload, *out, last_error_);
    }

    return true;
}

bool Gripper::moveToCountWithLimits(int32_t target_count,
                                    float max_speed_rpm,
                                    float max_current_amp,
                                    RealtimeStatus* out)
{
    MotionControlParameters params{};
    if (!readMotionControlParameters(params))
    {
        return false;
    }

    if (max_speed_rpm > 0.0f)
    {
        bool ok = false;
        const uint32_t raw_limit = convertSpeedLimitRpmToRaw(max_speed_rpm, ok);
        if (!ok)
        {
            last_error_ = "invalid max_speed_rpm";
            return false;
        }
        params.position_output_limit = raw_limit;
    }

    if (max_current_amp > 0.0f)
    {
        bool ok = false;
        const uint32_t raw_limit = convertCurrentLimitAmpToRaw(max_current_amp, ok);
        if (!ok)
        {
            last_error_ = "invalid max_current_amp";
            return false;
        }
        params.speed_output_limit = raw_limit;
    }

    if ((max_speed_rpm > 0.0f || max_current_amp > 0.0f) &&
        !writeMotionControlParametersTemp(params, nullptr))
    {
        return false;
    }

    return moveToCount(target_count, out);
}

bool Gripper::moveByCount(int32_t delta_count, RealtimeStatus* out)
{
    std::vector<uint8_t> payload;
    protocol::appendI32LE(payload, delta_count);

    protocol::Frame response;
    if (!transact(protocol::Command::MoveRelative, payload, response, true))
    {
        return false;
    }

    if (out)
    {
        return parseRealtimePayload(response.payload, *out, last_error_);
    }

    return true;
}

bool Gripper::goHomeShortest(RealtimeStatus* out)
{
    protocol::Frame response;
    if (!transact(protocol::Command::GoHomeShortest, {}, response, true))
    {
        return false;
    }

    if (out)
    {
        return parseRealtimePayload(response.payload, *out, last_error_);
    }

    return true;
}

bool Gripper::motorOff(RealtimeStatus* out)
{
    protocol::Frame response;
    if (!transact(protocol::Command::MotorOff, {}, response, true))
    {
        return false;
    }

    if (out)
    {
        return parseRealtimePayload(response.payload, *out, last_error_);
    }

    return true;
}

bool Gripper::reboot()
{
    protocol::Frame dummy;
    if (!transact(protocol::Command::Reboot, {}, dummy, false))
    {
        return false;
    }

    return true;
}

bool Gripper::setCurrentPositionAsZero(uint16_t& mechanical_offset)
{
    protocol::Frame response;
    if (!transact(protocol::Command::SetZeroPoint, {}, response, true))
    {
        return false;
    }

    if (response.payload.size() != 2)
    {
        last_error_ = "invalid zero-point payload length";
        return false;
    }

    mechanical_offset = protocol::readU16LE(response.payload.data());
    return true;
}

bool Gripper::restoreDefaultParameters()
{
    protocol::Frame response;
    if (!transact(protocol::Command::RestoreDefault, {}, response, true))
    {
        return false;
    }

    if (!response.payload.empty())
    {
        last_error_ = "invalid restore-default payload length";
        return false;
    }

    return true;
}

bool Gripper::brakeControl(BrakeAction action, uint8_t& brake_state)
{
    std::vector<uint8_t> payload;
    payload.push_back(static_cast<uint8_t>(action));

    protocol::Frame response;
    if (!transact(protocol::Command::BrakeControl, payload, response, true))
    {
        return false;
    }

    if (response.payload.size() != 1)
    {
        last_error_ = "invalid brake-control payload length";
        return false;
    }

    brake_state = response.payload[0];
    return true;
}

bool Gripper::brakeRelease(uint8_t& brake_state)
{
    return brakeControl(BrakeAction::Release, brake_state);
}

bool Gripper::brakeEngage(uint8_t& brake_state)
{
    return brakeControl(BrakeAction::Engage, brake_state);
}

bool Gripper::brakeReadState(uint8_t& brake_state)
{
    return brakeControl(BrakeAction::ReadState, brake_state);
}

bool Gripper::readUserParameters(UserParameters& out)
{
    protocol::Frame response;
    if (!transact(protocol::Command::ReadUserParams, {}, response, true))
    {
        return false;
    }

    return parseUserParametersPayload(response.payload, out, last_error_);
}

bool Gripper::writeUserParameters(const WritableUserParameters& in, UserParameters* out)
{
    if (!isConnected())
    {
        last_error_ = "device not connected";
        return false;
    }

    const std::vector<uint8_t> payload = buildWritableUserParametersPayload(in);
    const uint8_t request_device_address = device_address_;
    const uint8_t response_device_address = request_device_address;

    const uint8_t seq = next_sequence_++;
    const auto request = protocol::buildRequest(seq, request_device_address, protocol::Command::WriteUserParams, payload);

    std::size_t written = 0;
    while (written < request.size())
    {
        const int write_ret = transport_->writeBytes(request.data() + written, request.size() - written);
        if (write_ret <= 0)
        {
            last_error_ = "transport write failed";
            return false;
        }

        written += static_cast<std::size_t>(write_ret);
    }

    protocol::Frame response;
    if (!readResponseForDevice(response, seq, protocol::Command::WriteUserParams, response_device_address))
    {
        return false;
    }

    if (out)
    {
        if (!parseUserParametersPayload(response.payload, *out, last_error_))
        {
            return false;
        }
    }

    last_error_.clear();
    return true;
}

uint8_t Gripper::deviceAddress() const
{
    return device_address_;
}

bool Gripper::parseUserParametersPayload(const std::vector<uint8_t>& payload,
                                         UserParameters& out,
                                         std::string& error)
{
    if (payload.size() != 0x1A)
    {
        error = "invalid user-parameters payload length";
        return false;
    }

    out.electrical_angle_offset = protocol::readU16LE(&payload[0]);
    out.mechanical_angle_offset = protocol::readU16LE(&payload[2]);
    out.current_offset_u = protocol::readU16LE(&payload[4]);
    out.current_offset_v = protocol::readU16LE(&payload[6]);
    out.current_offset_w = protocol::readU16LE(&payload[8]);

    out.encoder_model = payload[10];
    out.invert_encoder_direction = (payload[11] != 0);
    out.enable_second_encoder = (payload[12] != 0);
    out.speed_filter_coeff = payload[13];

    out.device_address = payload[14];
    out.rs485_baudrate = static_cast<Rs485BaudrateCode>(payload[15]);
    out.can_baudrate = static_cast<CanBaudrateCode>(payload[16]);
    out.enable_canopen = (payload[17] != 0);

    out.max_bus_voltage_0p01v = protocol::readU16LE(&payload[18]);
    out.voltage_fault_delay_s = payload[20];

    out.max_bus_current_0p01a = protocol::readU16LE(&payload[21]);
    out.current_fault_delay_s = payload[23];

    out.max_temperature_c = payload[24];
    out.temperature_fault_delay_s = payload[25];

    return true;
}

std::vector<uint8_t> Gripper::buildWritableUserParametersPayload(const WritableUserParameters& in)
{
    std::vector<uint8_t> payload;
    payload.reserve(16);

    payload.push_back(in.encoder_model);
    payload.push_back(in.invert_encoder_direction ? 1 : 0);
    payload.push_back(in.enable_second_encoder ? 1 : 0);
    payload.push_back(in.speed_filter_coeff);

    payload.push_back(in.device_address);
    payload.push_back(static_cast<uint8_t>(in.rs485_baudrate));
    payload.push_back(static_cast<uint8_t>(in.can_baudrate));
    payload.push_back(in.enable_canopen ? 1 : 0);

    protocol::appendU16LE(payload, in.max_bus_voltage_0p01v);
    payload.push_back(in.voltage_fault_delay_s);

    protocol::appendU16LE(payload, in.max_bus_current_0p01a);
    payload.push_back(in.current_fault_delay_s);

    payload.push_back(in.max_temperature_c);
    payload.push_back(in.temperature_fault_delay_s);

    return payload;
}

bool Gripper::readMotionControlParameters(MotionControlParameters& out)
{
    protocol::Frame response;
    if (!transact(protocol::Command::ReadMotionParams, {}, response, true))
    {
        return false;
    }

    return parseMotionControlParametersPayload(response.payload, out, last_error_);
}

bool Gripper::writeMotionControlParametersTemp(const MotionControlParameters& in,
                                               MotionControlParameters* out)
{
    const std::vector<uint8_t> payload = buildMotionControlParametersPayload(in);

    protocol::Frame response;
    if (!transact(protocol::Command::WriteMotionParamsTemp, payload, response, true))
    {
        return false;
    }

    if (out)
    {
        return parseMotionControlParametersPayload(response.payload, *out, last_error_);
    }

    return true;
}

bool Gripper::writeMotionControlParametersSave(const MotionControlParameters& in,
                                               MotionControlParameters* out)
{
    const std::vector<uint8_t> payload = buildMotionControlParametersPayload(in);

    protocol::Frame response;
    if (!transact(protocol::Command::WriteMotionParamsSave, payload, response, true))
    {
        return false;
    }

    if (out)
    {
        return parseMotionControlParametersPayload(response.payload, *out, last_error_);
    }

    return true;
}

bool Gripper::parseMotionControlParametersPayload(const std::vector<uint8_t>& payload,
                                                  MotionControlParameters& out,
                                                  std::string& error)
{
    if (payload.size() != 0x18)
    {
        error = "invalid motion-control-parameters payload length";
        return false;
    }

    out.position_kp = protocol::readFloatLE(&payload[0]);
    out.position_ki = protocol::readFloatLE(&payload[4]);
    out.position_output_limit = protocol::readU32LE(&payload[8]);

    out.speed_kp = protocol::readFloatLE(&payload[12]);
    out.speed_ki = protocol::readFloatLE(&payload[16]);
    out.speed_output_limit = protocol::readU32LE(&payload[20]);

    return true;
}

std::vector<uint8_t> Gripper::buildMotionControlParametersPayload(const MotionControlParameters& in)
{
    std::vector<uint8_t> payload;
    payload.reserve(24);

    protocol::appendFloatLE(payload, in.position_kp);
    protocol::appendFloatLE(payload, in.position_ki);
    protocol::appendU32LE(payload, in.position_output_limit);

    protocol::appendFloatLE(payload, in.speed_kp);
    protocol::appendFloatLE(payload, in.speed_ki);
    protocol::appendU32LE(payload, in.speed_output_limit);

    return payload;
}

bool Gripper::readMotorHardwareParameters(MotorHardwareParameters& out)
{
    protocol::Frame response;
    if (!transact(protocol::Command::ReadMotorHwParams, {}, response, true))
    {
        return false;
    }

    return parseMotorHardwareParametersPayload(response.payload, out, last_error_);
}

bool Gripper::writeMotorHardwareParameters(const MotorHardwareParameters& in,
                                           MotorHardwareParameters* out)
{
    const std::vector<uint8_t> payload = buildMotorHardwareParametersPayload(in);

    protocol::Frame response;
    if (!transact(protocol::Command::WriteMotorHwParams, payload, response, true))
    {
        return false;
    }

    if (out)
    {
        return parseMotorHardwareParametersPayload(response.payload, *out, last_error_);
    }

    return true;
}

bool Gripper::parseMotorHardwareParametersPayload(const std::vector<uint8_t>& payload,
                                                  MotorHardwareParameters& out,
                                                  std::string& error)
{
    if (payload.size() != 0x1E)
    {
        error = "invalid motor-hardware-parameters payload length";
        return false;
    }

    const char* name_ptr = reinterpret_cast<const char*>(payload.data());
    std::size_t name_len = 0;
    while (name_len < 16 && name_ptr[name_len] != '\0')
    {
        ++name_len;
    }
    out.motor_name.assign(name_ptr, name_len);

    out.pole_pairs = payload[16];
    out.phase_resistance_ohm = protocol::readFloatLE(&payload[17]);
    out.phase_inductance_mh = protocol::readFloatLE(&payload[21]);
    out.torque_constant_nm = protocol::readFloatLE(&payload[25]);
    out.gear_ratio = payload[29];

    return true;
}

std::vector<uint8_t> Gripper::buildMotorHardwareParametersPayload(const MotorHardwareParameters& in)
{
    std::vector<uint8_t> payload;
    payload.reserve(30);

    for (std::size_t i = 0; i < 16; ++i)
    {
        if (i < in.motor_name.size())
        {
            payload.push_back(static_cast<uint8_t>(in.motor_name[i]));
        }
        else
        {
            payload.push_back(0);
        }
    }

    payload.push_back(in.pole_pairs);
    protocol::appendFloatLE(payload, in.phase_resistance_ohm);
    protocol::appendFloatLE(payload, in.phase_inductance_mh);
    protocol::appendFloatLE(payload, in.torque_constant_nm);
    payload.push_back(in.gear_ratio);

    return payload;
}

bool Gripper::startEncoderCalibration(RealtimeStatus* out)
{
    protocol::Frame response;
    if (!transact(protocol::Command::EncoderCalib, {}, response, true))
    {
        return false;
    }

    if (out)
    {
        return parseRealtimePayload(response.payload, *out, last_error_);
    }

    return true;
}

bool Gripper::startEncoderCalibrationAndWait(int wait_ms,
                                             int poll_interval_ms,
                                             RealtimeStatus& out)
{
    RealtimeStatus first{};
    if (!startEncoderCalibration(&first))
    {
        return false;
    }

    const int loops = (poll_interval_ms > 0) ? (wait_ms / poll_interval_ms) : 0;

    for (int i = 0; i < loops; ++i)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));

        if (!readRealtime(out))
        {
            return false;
        }
    }

    return true;
}

bool Gripper::transact(protocol::Command cmd,
                       const std::vector<uint8_t>& payload,
                       protocol::Frame& response,
                       bool expect_response)
{
    if (!isConnected())
    {
        last_error_ = "device not connected";
        return false;
    }

    const uint8_t seq = next_sequence_++;
    const auto request = protocol::buildRequest(seq, device_address_, cmd, payload);

    std::size_t written = 0;
    while (written < request.size())
    {
        const int write_ret = transport_->writeBytes(request.data() + written, request.size() - written);
        if (write_ret <= 0)
        {
            last_error_ = "transport write failed";
            return false;
        }

        written += static_cast<std::size_t>(write_ret);
    }

    if (!expect_response || device_address_ == 0x00)
    {
        last_error_.clear();
        return true;
    }

    if (!readResponse(response, seq, cmd))
    {
        return false;
    }

    last_error_.clear();
    return true;
}

bool Gripper::readResponse(protocol::Frame& frame,
                           uint8_t expected_sequence,
                           protocol::Command expected_command)
{
    return readResponseForDevice(frame, expected_sequence, expected_command, device_address_);
}

bool Gripper::readResponseForDevice(protocol::Frame& frame,
                                    uint8_t expected_sequence,
                                    protocol::Command expected_command,
                                    uint8_t expected_device_address)
{
    if (!transport_)
    {
        last_error_ = "transport not ready";
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms_);

    std::string error;
    std::string last_skipped_reason = "read timeout";

    while (remainingMs(deadline) > 0)
    {
        uint8_t header = 0;
        if (!readExactUntil(*transport_, &header, 1, deadline, error))
        {
            last_error_ = error;
            return false;
        }

        if (header != protocol::kSlaveHeader)
        {
            last_skipped_reason = "skipped non-slave header byte";
            continue;
        }

        uint8_t fixed_part[4]{};
        if (!readExactUntil(*transport_, fixed_part, sizeof(fixed_part), deadline, error))
        {
            last_error_ = error;
            return false;
        }

        const uint8_t payload_len = fixed_part[3];
        if (payload_len > 248)
        {
            last_skipped_reason = "skipped frame with invalid payload length";
            continue;
        }

        std::vector<uint8_t> raw;
        raw.reserve(5 + payload_len + 2);
        raw.push_back(header);
        raw.push_back(fixed_part[0]);
        raw.push_back(fixed_part[1]);
        raw.push_back(fixed_part[2]);
        raw.push_back(fixed_part[3]);

        std::vector<uint8_t> tail(static_cast<std::size_t>(payload_len) + 2);
        if (!readExactUntil(*transport_, tail.data(), tail.size(), deadline, error))
        {
            last_error_ = error;
            return false;
        }

        raw.insert(raw.end(), tail.begin(), tail.end());

        protocol::Frame candidate;
        if (!protocol::parseFrame(raw, candidate, &error))
        {
            last_skipped_reason = "skipped malformed frame: " + error;
            continue;
        }

        if (candidate.sequence != expected_sequence)
        {
            last_skipped_reason = "skipped unexpected response sequence";
            continue;
        }

        if (candidate.command != static_cast<uint8_t>(expected_command))
        {
            last_skipped_reason = "skipped unexpected response command";
            continue;
        }

        if (!isExpectedResponseDeviceFor(expected_device_address, candidate.device))
        {
            last_skipped_reason = "skipped unexpected response device";
            continue;
        }

        frame = std::move(candidate);
        return true;
    }

    last_error_ = last_skipped_reason;
    return false;
}

bool Gripper::isExpectedResponseDevice(uint8_t response_device) const
{
    return isExpectedResponseDeviceFor(device_address_, response_device);
}

bool Gripper::isExpectedResponseDeviceFor(uint8_t requested_device_address,
                                          uint8_t response_device)
{
    if (requested_device_address == 0xFF)
    {
        return response_device != 0x00 && response_device != 0xFF;
    }

    return response_device == requested_device_address;
}

bool Gripper::parseRealtimePayload(const std::vector<uint8_t>& payload,
                                   RealtimeStatus& out,
                                   std::string& error)
{
    if (payload.size() != 0x16)
    {
        error = "invalid realtime payload length";
        return false;
    }

    out.single_turn_raw = protocol::readU16LE(&payload[0]);
    out.single_turn_deg = static_cast<float>(out.single_turn_raw) * (360.0f / 16384.0f);

    out.multi_turn_count = protocol::readI32LE(&payload[2]);
    out.multi_turn_deg = static_cast<float>(out.multi_turn_count) * (360.0f / 16384.0f);

    out.speed_raw = protocol::readI32LE(&payload[6]);
    out.speed_rpm = static_cast<float>(out.speed_raw) * 0.01f;

    out.q_current_raw = protocol::readI32LE(&payload[10]);
    out.q_current_amp = static_cast<float>(out.q_current_raw) * 0.001f;

    out.bus_voltage_raw = protocol::readU16LE(&payload[14]);
    out.bus_voltage_v = static_cast<float>(out.bus_voltage_raw) * 0.01f;

    out.bus_current_raw = protocol::readU16LE(&payload[16]);
    out.bus_current_a = static_cast<float>(out.bus_current_raw) * 0.01f;

    out.temperature_c = payload[18];
    out.run_state = payload[19];
    out.motor_enabled = (payload[20] != 0);
    out.fault_code = payload[21];

    return true;
}
} // namespace gripper
