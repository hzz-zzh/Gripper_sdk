#include "core/gripper.h"
#include "core/gripper_device.h"
#include "protocol/protocol.h"
#include "transport/i_transport.h"

#include <algorithm>
#include <cstdint>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using gripper::protocol::Command;
using gripper::protocol::Frame;

struct TestFailure : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

void expectTrue(bool value, const std::string& message)
{
    if (!value)
    {
        throw TestFailure(message);
    }
}

template <typename T, typename U>
void expectEqual(const T& lhs, const U& rhs, const std::string& message)
{
    if (!(lhs == rhs))
    {
        throw TestFailure(message);
    }
}

std::vector<uint8_t> makeRealtimePayload(int32_t multi_turn_count,
                                         int32_t speed_raw,
                                         int32_t q_current_raw,
                                         uint16_t single_turn_raw = 0,
                                         uint16_t bus_voltage_raw = 3200,
                                         uint16_t bus_current_raw = 0,
                                         uint8_t temperature_c = 25,
                                         uint8_t run_state = 4,
                                         bool motor_enabled = true,
                                         uint8_t fault_code = 0)
{
    std::vector<uint8_t> payload;
    payload.reserve(0x16);
    gripper::protocol::appendU16LE(payload, single_turn_raw);
    gripper::protocol::appendI32LE(payload, multi_turn_count);
    gripper::protocol::appendI32LE(payload, speed_raw);
    gripper::protocol::appendI32LE(payload, q_current_raw);
    gripper::protocol::appendU16LE(payload, bus_voltage_raw);
    gripper::protocol::appendU16LE(payload, bus_current_raw);
    payload.push_back(temperature_c);
    payload.push_back(run_state);
    payload.push_back(motor_enabled ? 1 : 0);
    payload.push_back(fault_code);
    return payload;
}

std::vector<uint8_t> makeRealtimeResponse(uint8_t seq,
                                          uint8_t device,
                                          Command command,
                                          int32_t multi_turn_count,
                                          int32_t speed_raw,
                                          int32_t q_current_raw,
                                          uint16_t single_turn_raw = 0,
                                          uint16_t bus_voltage_raw = 3200,
                                          uint16_t bus_current_raw = 0,
                                          uint8_t temperature_c = 25,
                                          uint8_t run_state = 4,
                                          bool motor_enabled = true,
                                          uint8_t fault_code = 0)
{
    return gripper::protocol::buildFrame(
        gripper::protocol::kSlaveHeader,
        seq,
        device,
        static_cast<uint8_t>(command),
        makeRealtimePayload(multi_turn_count,
                            speed_raw,
                            q_current_raw,
                            single_turn_raw,
                            bus_voltage_raw,
                            bus_current_raw,
                            temperature_c,
                            run_state,
                            motor_enabled,
                            fault_code));
}

std::vector<uint8_t> makeRawBadCrcRealtimeResponse(uint8_t seq, uint8_t device, Command command)
{
    auto raw = makeRealtimeResponse(seq, device, command, 111, 0, 0);
    raw.back() ^= 0x5A;
    return raw;
}

class ScriptedTransport : public gripper::ITransport
{
public:
    using RequestHandler = std::function<void(const Frame&, ScriptedTransport&)>;

    explicit ScriptedTransport(std::size_t max_write_chunk = 0)
        : max_write_chunk_(max_write_chunk)
    {
    }

    bool open() override
    {
        open_ = true;
        return true;
    }

    void close() override
    {
        open_ = false;
    }

    bool isOpen() const override
    {
        return open_;
    }

    int writeBytes(const uint8_t* data, std::size_t size) override
    {
        if (!open_)
        {
            return -1;
        }

        const std::size_t chunk = (max_write_chunk_ == 0) ? size : std::min(size, max_write_chunk_);
        ++write_call_count_;
        written_bytes_.insert(written_bytes_.end(), data, data + chunk);
        pending_request_bytes_.insert(pending_request_bytes_.end(), data, data + chunk);
        consumePendingRequests();
        return static_cast<int>(chunk);
    }

    int readBytes(uint8_t* data, std::size_t size, int /*timeout_ms*/) override
    {
        if (!open_)
        {
            return -1;
        }

        if (read_queue_.empty())
        {
            return 0;
        }

        const std::size_t count = std::min(size, read_queue_.size());
        for (std::size_t i = 0; i < count; ++i)
        {
            data[i] = read_queue_.front();
            read_queue_.pop_front();
        }

        return static_cast<int>(count);
    }

    void setHandler(RequestHandler handler)
    {
        handler_ = std::move(handler);
    }

    void enqueueRaw(const std::vector<uint8_t>& raw)
    {
        read_queue_.insert(read_queue_.end(), raw.begin(), raw.end());
    }

    void enqueueFrame(uint8_t seq,
                      uint8_t device,
                      Command command,
                      const std::vector<uint8_t>& payload)
    {
        enqueueRaw(gripper::protocol::buildFrame(gripper::protocol::kSlaveHeader,
                                                 seq,
                                                 device,
                                                 static_cast<uint8_t>(command),
                                                 payload));
    }

    int writeCallCount() const
    {
        return write_call_count_;
    }

    const std::vector<uint8_t>& writtenBytes() const
    {
        return written_bytes_;
    }

    const std::vector<Command>& requestHistory() const
    {
        return request_history_;
    }

private:
    void consumePendingRequests()
    {
        while (pending_request_bytes_.size() >= 5)
        {
            const std::size_t full_size = 5 + static_cast<std::size_t>(pending_request_bytes_[4]) + 2;
            if (pending_request_bytes_.size() < full_size)
            {
                return;
            }

            std::vector<uint8_t> raw(pending_request_bytes_.begin(),
                                     pending_request_bytes_.begin() + static_cast<std::ptrdiff_t>(full_size));
            pending_request_bytes_.erase(pending_request_bytes_.begin(),
                                         pending_request_bytes_.begin() + static_cast<std::ptrdiff_t>(full_size));

            Frame frame;
            std::string error;
            expectTrue(gripper::protocol::parseFrame(raw, frame, &error),
                       "failed to parse request frame in test transport: " + error);
            request_history_.push_back(static_cast<Command>(frame.command));

            if (handler_)
            {
                handler_(frame, *this);
            }
        }
    }

private:
    bool open_ = false;
    std::size_t max_write_chunk_ = 0;
    int write_call_count_ = 0;
    std::vector<uint8_t> written_bytes_;
    std::vector<uint8_t> pending_request_bytes_;
    std::deque<uint8_t> read_queue_;
    std::vector<Command> request_history_;
    RequestHandler handler_;
};

void test_transport_partial_write_and_skip_bad_frames()
{
    auto transport = std::make_unique<ScriptedTransport>(3);
    auto* transport_ptr = transport.get();

    transport_ptr->setHandler([](const Frame& request, ScriptedTransport& io) {
        expectEqual(request.command, static_cast<uint8_t>(Command::ReadRealtime),
                    "unexpected command in realtime test");

        io.enqueueRaw(makeRawBadCrcRealtimeResponse(request.sequence,
                                                   request.device,
                                                   Command::ReadRealtime));
        io.enqueueRaw(makeRealtimeResponse(request.sequence,
                                          static_cast<uint8_t>(request.device + 1),
                                          Command::ReadRealtime,
                                          200,
                                          1234,
                                          500));
        io.enqueueRaw(makeRealtimeResponse(request.sequence,
                                          request.device,
                                          Command::ReadRealtime,
                                          321,
                                          2345,
                                          678));
    });

    gripper::Gripper motor(0x01, std::move(transport));
    expectTrue(motor.connect("ignored", 115200), "connect failed for transport test");

    gripper::RealtimeStatus status{};
    expectTrue(motor.readRealtime(status), "readRealtime should succeed after skipping bad frames");
    expectEqual(status.multi_turn_count, 321, "unexpected multi_turn_count");
    expectEqual(status.speed_raw, 2345, "unexpected speed_raw");
    expectEqual(status.q_current_raw, 678, "unexpected q_current_raw");
    expectTrue(transport_ptr->writeCallCount() > 1, "partial-write path was not exercised");
}

void test_device_profile_and_initialize_with_mock_transport()
{
    gripper::GripperDeviceConfig config;
    config.device_address = 0x01;
    config.position_profile.open_position_count = -1000;
    config.position_profile.close_position_count = 200;

    auto transport = std::make_unique<ScriptedTransport>();
    auto* transport_ptr = transport.get();

    int realtime_read_count = 0;
    int32_t last_move_absolute_target = 0;

    transport_ptr->setHandler([&](const Frame& request, ScriptedTransport& io) {
        const auto cmd = static_cast<Command>(request.command);
        switch (cmd)
        {
            case Command::ClearFault:
                io.enqueueFrame(request.sequence, request.device, cmd, {0x00});
                break;

            case Command::WriteSpeed:
                io.enqueueRaw(makeRealtimeResponse(request.sequence,
                                                  request.device,
                                                  cmd,
                                                  100,
                                                  500,
                                                  100));
                break;

            case Command::ReadRealtime:
                ++realtime_read_count;
                if (realtime_read_count <= 3)
                {
                    io.enqueueRaw(makeRealtimeResponse(request.sequence,
                                                      request.device,
                                                      cmd,
                                                      100,
                                                      0,
                                                      1200));
                }
                else
                {
                    io.enqueueRaw(makeRealtimeResponse(request.sequence,
                                                      request.device,
                                                      cmd,
                                                      -100,
                                                      0,
                                                      0));
                }
                break;

            case Command::MotorOff:
                io.enqueueRaw(makeRealtimeResponse(request.sequence,
                                                  request.device,
                                                  cmd,
                                                  100,
                                                  0,
                                                  0,
                                                  0,
                                                  3200,
                                                  0,
                                                  25,
                                                  0,
                                                  false,
                                                  0));
                break;

            case Command::SetZeroPoint:
            {
                std::vector<uint8_t> payload;
                gripper::protocol::appendU16LE(payload, 0x1234);
                io.enqueueFrame(request.sequence, request.device, cmd, payload);
                break;
            }

            case Command::MoveRelative:
                io.enqueueRaw(makeRealtimeResponse(request.sequence,
                                                  request.device,
                                                  cmd,
                                                  -100,
                                                  0,
                                                  0));
                break;

            case Command::MoveAbsolute:
                last_move_absolute_target = gripper::protocol::readI32LE(request.payload.data());
                io.enqueueRaw(makeRealtimeResponse(request.sequence,
                                                  request.device,
                                                  cmd,
                                                  last_move_absolute_target,
                                                  0,
                                                  0));
                break;

            default:
                throw TestFailure("unexpected command in initialize test");
        }
    });

    gripper::GripperDevice device(config, std::move(transport));
    expectTrue(device.connect(), "device connect failed");

    expectEqual(device.percentToCount(0.0f), 200, "0 percent should map to close count");
    expectEqual(device.percentToCount(100.0f), -1000, "100 percent should map to open count");
    expectEqual(device.percentToCount(25.0f), -100, "25 percent mapping mismatch");
    expectEqual(static_cast<int>(device.countToPercent(-100)), 25, "countToPercent mismatch");

    gripper::GripperInitializeConfig init_cfg;
    init_cfg.poll_interval_ms = 1;
    init_cfg.timeout_ms = 100;
    init_cfg.detect_consecutive_samples = 3;
    init_cfg.backoff_count_after_zero = 100;

    gripper::GripperInitializeResult result{};
    expectTrue(device.initialize(init_cfg, &result), "initialize should succeed with mock transport");
    expectTrue(device.isInitialized(), "device should be initialized");
    expectTrue(result.limit_detected, "limit should be detected");
    expectTrue(result.zero_set, "zero should be set");
    expectTrue(result.backoff_done, "backoff should be executed");
    expectEqual(result.mechanical_offset, static_cast<uint16_t>(0x1234), "mechanical offset mismatch");
    expectEqual(result.final_status.multi_turn_count, -100, "final status count mismatch");

    expectTrue(device.open(), "open command should succeed after initialization");
    expectEqual(last_move_absolute_target, -1000, "open should use profile open count");

    expectTrue(device.moveToPercent(25.0f), "moveToPercent should succeed after initialization");
    expectEqual(last_move_absolute_target, -100, "moveToPercent should use profile mapping");
}

} // namespace

int main()
{
    try
    {
        test_transport_partial_write_and_skip_bad_frames();
        test_device_profile_and_initialize_with_mock_transport();
        std::cout << "All tests passed\n";
        return 0;
    }
    catch (const TestFailure& e)
    {
        std::cerr << "Test failure: " << e.what() << '\n';
        return 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Unexpected exception: " << e.what() << '\n';
        return 1;
    }
}
