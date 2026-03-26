#include "core/gripper.h"
#include "transport/rs485_transport.h"

#include <chrono>
#include <utility>

namespace gripper
{
namespace
{
bool readExact(ITransport& transport,
               uint8_t* buffer,
               std::size_t size,
               int timeout_ms,
               std::string& error)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    std::size_t received = 0;

    while (received < size)
    {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
        {
            error = "read timeout";
            return false;
        }

        const int remain_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());

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
}

Gripper::Gripper(uint8_t device_address)
    : transport_(nullptr),
      device_address_(device_address),
      next_sequence_(0),
      timeout_ms_(200),
      last_error_()
{
}

bool Gripper::connect(const std::string& port_name, int baudrate)
{
    transport_ = std::make_unique<Rs485Transport>(port_name, baudrate);
    if (!transport_->open())
    {
        last_error_ = "failed to open rs485 port";
        transport_.reset();
        return false;
    }

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

    const int write_ret = transport_->writeBytes(request.data(), request.size());
    if (write_ret != static_cast<int>(request.size()))
    {
        last_error_ = "transport write failed";
        return false;
    }

    if (!expect_response || device_address_ == 0x00)
    {
        return true;
    }

    if (!readResponse(response))
    {
        return false;
    }

    if (response.header != protocol::kSlaveHeader)
    {
        last_error_ = "invalid response header";
        return false;
    }

    if (response.sequence != seq)
    {
        last_error_ = "response sequence mismatch";
        return false;
    }

    if (response.command != static_cast<uint8_t>(cmd))
    {
        last_error_ = "response command mismatch";
        return false;
    }

    return true;
}

bool Gripper::readResponse(protocol::Frame& frame)
{
    if (!transport_)
    {
        last_error_ = "transport not ready";
        return false;
    }

    std::string error;
    uint8_t header = 0;

    do
    {
        if (!readExact(*transport_, &header, 1, timeout_ms_, error))
        {
            last_error_ = error;
            return false;
        }
    } while (header != protocol::kSlaveHeader);

    uint8_t fixed_part[4]{};
    if (!readExact(*transport_, fixed_part, sizeof(fixed_part), timeout_ms_, error))
    {
        last_error_ = error;
        return false;
    }

    const uint8_t payload_len = fixed_part[3];
    std::vector<uint8_t> raw;
    raw.reserve(5 + payload_len + 2);

    raw.push_back(header);
    raw.push_back(fixed_part[0]);
    raw.push_back(fixed_part[1]);
    raw.push_back(fixed_part[2]);
    raw.push_back(fixed_part[3]);

    std::vector<uint8_t> tail(static_cast<std::size_t>(payload_len) + 2);
    if (!readExact(*transport_, tail.data(), tail.size(), timeout_ms_, error))
    {
        last_error_ = error;
        return false;
    }

    raw.insert(raw.end(), tail.begin(), tail.end());

    if (!protocol::parseFrame(raw, frame, &error))
    {
        last_error_ = error;
        return false;
    }

    return true;
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
}