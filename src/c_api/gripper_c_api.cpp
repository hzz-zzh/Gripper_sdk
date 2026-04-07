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
bool copy_status(const gripper::GripperStatus& in, gripper_status_t* out)
{
    if (out == nullptr)
    {
        return false;
    }

    out->opening_mm = in.opening_mm;
    out->opening_speed_mm_s = in.opening_speed_mm_s;
    out->q_current_amp = in.q_current_amp;
    out->bus_voltage_v = in.bus_voltage_v;
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
    out->limit_opening_mm_before_zero = in.limit_opening_mm_before_zero;
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
} // namespace

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

    config->search_speed_mm_s = 50.0f;
    config->search_direction = +1;

    config->poll_interval_ms = 20;
    config->timeout_ms = 5000;

    config->speed_epsilon_mm_s = 0.25f;
    config->current_threshold_a = 0.6f;
    config->position_epsilon_mm = 0.05f;
    config->detect_consecutive_samples = 4;

    config->clear_fault_before_start = 1;
    config->set_zero_after_detect = 1;
    config->backoff_after_zero_mm = 2.0f;
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
    cpp_config.search_speed_mm_s = config->search_speed_mm_s;
    cpp_config.search_direction = config->search_direction;
    cpp_config.poll_interval_ms = config->poll_interval_ms;
    cpp_config.timeout_ms = config->timeout_ms;
    cpp_config.speed_epsilon_mm_s = config->speed_epsilon_mm_s;
    cpp_config.current_threshold_a = config->current_threshold_a;
    cpp_config.position_epsilon_mm = config->position_epsilon_mm;
    cpp_config.detect_consecutive_samples = config->detect_consecutive_samples;
    cpp_config.clear_fault_before_start = (config->clear_fault_before_start != 0);
    cpp_config.set_zero_after_detect = (config->set_zero_after_detect != 0);
    cpp_config.backoff_after_zero_mm = config->backoff_after_zero_mm;

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

int gripper_move_to_opening_mm(gripper_handle_t* handle, float opening_mm)
{
    if (handle == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    if (!handle->device.moveToOpeningMm(opening_mm))
    {
        set_error_from_device(handle);
        return GRIPPER_API_ERROR;
    }

    set_error(handle, "");
    return GRIPPER_API_OK;
}

int gripper_move_to_opening_mm_with_limits(gripper_handle_t* handle,
                                           float opening_mm,
                                           float max_speed_mm_s,
                                           float max_current_amp)
{
    if (handle == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    if (!handle->device.moveToOpeningMmWithLimits(opening_mm,
                                                  max_speed_mm_s,
                                                  max_current_amp))
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

int gripper_read_status(gripper_handle_t* handle, gripper_status_t* out_status)
{
    if (handle == nullptr || out_status == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    gripper::GripperStatus status{};
    if (!handle->device.readStatus(status))
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

    if (!handle->device.reboot())
    {
        set_error_from_device(handle);
        return GRIPPER_API_ERROR;
    }

    set_error(handle, "");
    return GRIPPER_API_OK;
}

int gripper_set_communication_config(gripper_handle_t* handle,
                                   uint8_t new_device_address,
                                   int new_baudrate)
{
    if (handle == nullptr)
    {
        return GRIPPER_API_INVALID_ARGUMENT;
    }

    if (!handle->device.setCommunicationConfig(new_device_address, new_baudrate))
    {
        set_error_from_device(handle);
        return GRIPPER_API_ERROR;
    }

    set_error(handle, "");
    return GRIPPER_API_OK;
}

float gripper_get_min_opening_mm(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return 0.0f;
    }

    return handle->device.minOpeningMm();
}

float gripper_get_max_opening_mm(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return 0.0f;
    }

    return handle->device.maxOpeningMm();
}

const char* gripper_get_last_error(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return "invalid handle";
    }

    return handle->last_error.c_str();
}
} // extern "C"