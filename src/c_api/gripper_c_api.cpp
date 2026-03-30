#include "c_api/gripper_c_api.h"
#include "core/gripper_device.h"

#include <memory>
#include <new>
#include <string>

struct gripper_handle
{
    gripper::GripperDevice device;
    std::string last_error;

    explicit gripper_handle(const gripper::GripperDeviceConfig& config)
        : device(config), last_error()
    {
    }
};

namespace
{
bool copy_status(const gripper::RealtimeStatus& in, gripper_realtime_status_t* out)
{
    if (out == nullptr)
    {
        return false;
    }

    out->single_turn_raw = in.single_turn_raw;
    out->single_turn_deg = in.single_turn_deg;

    out->multi_turn_count = in.multi_turn_count;
    out->multi_turn_deg = in.multi_turn_deg;

    out->speed_raw = in.speed_raw;
    out->speed_rpm = in.speed_rpm;

    out->q_current_raw = in.q_current_raw;
    out->q_current_amp = in.q_current_amp;

    out->bus_voltage_raw = in.bus_voltage_raw;
    out->bus_voltage_v = in.bus_voltage_v;

    out->bus_current_raw = in.bus_current_raw;
    out->bus_current_a = in.bus_current_a;

    out->temperature_c = in.temperature_c;
    out->run_state = in.run_state;
    out->motor_enabled = in.motor_enabled ? 1 : 0;
    out->fault_code = in.fault_code;

    return true;
}

bool copy_initialize_result(const gripper::GripperInitializeResult& in,
                            gripper_initialize_result_t* out)
{
    if (out == nullptr)
    {
        return false;
    }

    out->limit_detected = in.limit_detected ? 1 : 0;
    out->zero_set = in.zero_set ? 1 : 0;
    out->backoff_done = in.backoff_done ? 1 : 0;

    out->detect_samples = in.detect_samples;
    out->limit_count_before_zero = in.limit_count_before_zero;
    out->mechanical_offset = in.mechanical_offset;

    copy_status(in.final_status, &out->final_status);
    return true;
}

void set_error(gripper_handle* handle, const std::string& err)
{
    if (handle != nullptr)
    {
        handle->last_error = err;
    }
}

void set_error_from_device(gripper_handle* handle)
{
    if (handle != nullptr)
    {
        handle->last_error = handle->device.lastError();
    }
}
}

extern "C"
{
gripper_handle_t* gripper_create(const gripper_config_t* config)
{
    if (config == nullptr || config->port_name == nullptr)
    {
        return nullptr;
    }

    gripper::GripperDeviceConfig cpp_config;
    cpp_config.port_name = config->port_name;
    cpp_config.baudrate = config->baudrate;
    cpp_config.device_address = config->device_address;
    cpp_config.timeout_ms = config->timeout_ms;
    try
    {
        return new gripper_handle(cpp_config);
    }
    catch (const std::bad_alloc&)
    {
        return nullptr;
    }
}

void gripper_destroy(gripper_handle_t* handle)
{
    delete handle;
}

int gripper_connect(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    if (!handle->device.connect())
    {
        set_error_from_device(handle);
        return GRIPPER_API_ERROR;
    }

    set_error(handle, "");
    return GRIPPER_API_OK;
}

void gripper_disconnect(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return;
    }

    handle->device.disconnect();
}

int gripper_is_connected(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return 0;
    }

    return handle->device.isConnected() ? 1 : 0;
}

void gripper_initialize_config_init(gripper_initialize_config_t* config)
{
    if (config == nullptr)
    {
        return;
    }

    config->search_speed_rpm = 5.0f;
    config->search_direction = +1;

    config->poll_interval_ms = 20;
    config->timeout_ms = 5000;

    config->speed_epsilon_rpm = 0.5f;
    config->current_threshold_a = 0.6f;
    config->position_epsilon_count = 5;
    config->detect_consecutive_samples = 4;

    config->clear_fault_before_start = 1;
    config->set_zero_after_detect = 1;
    config->backoff_count_after_zero = 200;
}

int gripper_initialize(gripper_handle_t* handle,
                       const gripper_initialize_config_t* config,
                       gripper_initialize_result_t* out_result)
{
    if (handle == nullptr || config == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    gripper::GripperInitializeConfig cpp_config;
    cpp_config.search_speed_rpm = config->search_speed_rpm;
    cpp_config.search_direction = config->search_direction;

    cpp_config.poll_interval_ms = config->poll_interval_ms;
    cpp_config.timeout_ms = config->timeout_ms;

    cpp_config.speed_epsilon_rpm = config->speed_epsilon_rpm;
    cpp_config.current_threshold_a = config->current_threshold_a;
    cpp_config.position_epsilon_count = config->position_epsilon_count;
    cpp_config.detect_consecutive_samples = config->detect_consecutive_samples;

    cpp_config.clear_fault_before_start = (config->clear_fault_before_start != 0);
    cpp_config.set_zero_after_detect = (config->set_zero_after_detect != 0);
    cpp_config.backoff_count_after_zero = config->backoff_count_after_zero;

    gripper::GripperInitializeResult cpp_result{};
    if (!handle->device.initialize(cpp_config, &cpp_result))
    {
        set_error_from_device(handle);
        return GRIPPER_API_ERROR;
    }

    if (out_result != nullptr)
    {
        copy_initialize_result(cpp_result, out_result);
    }

    set_error(handle, "");
    return GRIPPER_API_OK;
}

int gripper_is_initialized(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return 0;
    }

    return handle->device.isInitialized() ? 1 : 0;
}

int gripper_move_to_position(gripper_handle_t* handle, int32_t target_position)
{
    if (handle == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    if (!handle->device.moveToPosition(target_position))
    {
        set_error_from_device(handle);
        return GRIPPER_API_ERROR;
    }

    set_error(handle, "");
    return GRIPPER_API_OK;
}

int gripper_move_relative(gripper_handle_t* handle, int32_t delta_position)
{
    if (handle == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    if (!handle->device.moveRelative(delta_position))
    {
        set_error_from_device(handle);
        return GRIPPER_API_ERROR;
    }

    set_error(handle, "");
    return GRIPPER_API_OK;
}

int gripper_open(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    if (!handle->device.open())
    {
        set_error_from_device(handle);
        return GRIPPER_API_ERROR;
    }

    set_error(handle, "");
    return GRIPPER_API_OK;
}

int gripper_close(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    if (!handle->device.close())
    {
        set_error_from_device(handle);
        return GRIPPER_API_ERROR;
    }

    set_error(handle, "");
    return GRIPPER_API_OK;
}

int gripper_move_to_percent(gripper_handle_t* handle, float percent)
{
    if (handle == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    if (!handle->device.moveToPercent(percent))
    {
        set_error_from_device(handle);
        return GRIPPER_API_ERROR;
    }

    set_error(handle, "");
    return GRIPPER_API_OK;
}

int gripper_stop(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    if (!handle->device.stop())
    {
        set_error_from_device(handle);
        return GRIPPER_API_ERROR;
    }

    set_error(handle, "");
    return GRIPPER_API_OK;
}

int gripper_read_realtime(gripper_handle_t* handle, gripper_realtime_status_t* out_status)
{
    if (handle == nullptr || out_status == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    gripper::RealtimeStatus status{};
    if (!handle->device.readRealtime(status))
    {
        set_error_from_device(handle);
        return GRIPPER_API_ERROR;
    }

    copy_status(status, out_status);
    set_error(handle, "");
    return GRIPPER_API_OK;
}

int gripper_reboot(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    if (!handle->device.motor().reboot())
    {
        handle->last_error = handle->device.motor().lastError();
        return GRIPPER_API_ERROR;
    }

    handle->device.invalidateInitialization();
    set_error(handle, "");
    return GRIPPER_API_OK;
}

const char* gripper_get_last_error(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return "invalid handle";
    }

    return handle->last_error.c_str();
}
}