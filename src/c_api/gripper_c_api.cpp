#include "c_api/gripper_c_api.h"
#include "core/gripper_device.h"

#include <memory>
#include <new>
#include <string>

struct gripper_handle
{
    gripper::GripperDevice device;
    std::string last_error;
    gripper_error_code_t last_error_code;

    explicit gripper_handle(const gripper::GripperDeviceConfig& config)
        : device(config), last_error(), last_error_code(GRIPPER_OK)
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

bool starts_with(const std::string& text, const char* prefix)
{
    const std::string prefix_str(prefix);
    return text.rfind(prefix_str, 0) == 0;
}

gripper_error_code_t map_error_to_code(const std::string& err)
{
    if (err.empty())
    {
        return GRIPPER_OK;
    }

    if (err == "communication config may have been applied; reconnect with new address/baudrate")
    {
        return GRIPPER_ERR_COMM_CONFIG_APPLIED;
    }

    if (err == "invalid handle")
    {
        return GRIPPER_ERR_INVALID_ARGUMENT;
    }

    if (starts_with(err, "invalid initialize config:"))
    {
        return GRIPPER_ERR_INVALID_CONFIG;
    }

    if (err == "invalid max_speed_rpm" ||
        err == "invalid max_current_amp" ||
        err == "invalid device address: expected 1~254" ||
        err == "invalid RS485 baudrate: expected one of 9600/19200/38400/57600/115200/460800/921600")
    {
        return GRIPPER_ERR_INVALID_ARGUMENT;
    }

    if (err == "device not connected")
    {
        return GRIPPER_ERR_NOT_CONNECTED;
    }

    if (err == "gripper not initialized")
    {
        return GRIPPER_ERR_NOT_INITIALIZED;
    }

    if (err == "read timeout" || err == "initialize timeout")
    {
        return GRIPPER_ERR_TIMEOUT;
    }

    if (err == "failed to open transport" ||
        err == "failed to open rs485 port" ||
        err == "transport not ready" ||
        err == "transport write failed" ||
        err == "transport read failed")
    {
        return GRIPPER_ERR_TRANSPORT;
    }

    if (starts_with(err, "invalid version payload length") ||
        starts_with(err, "invalid clear-fault payload length") ||
        starts_with(err, "invalid zero-point payload length") ||
        starts_with(err, "invalid restore-default payload length") ||
        starts_with(err, "invalid brake-control payload length") ||
        starts_with(err, "invalid user-parameters payload length") ||
        starts_with(err, "invalid motion-control-parameters payload length") ||
        starts_with(err, "invalid motor-hardware-parameters payload length") ||
        starts_with(err, "invalid realtime payload length") ||
        err == "skipped frame with invalid payload length" ||
        starts_with(err, "skipped malformed frame:"))
    {
        return GRIPPER_ERR_PROTOCOL;
    }

    if (err == "skipped non-slave header byte" ||
        err == "skipped unexpected response sequence" ||
        err == "skipped unexpected response command" ||
        err == "skipped unexpected response device")
    {
        return GRIPPER_ERR_BAD_RESPONSE;
    }

    if (err == "target opening_mm is outside the valid geometry range")
    {
        return GRIPPER_ERR_OUT_OF_RANGE;
    }

    if (starts_with(err, "fault occurred during"))
    {
        return GRIPPER_ERR_DEVICE_FAULT;
    }

    return GRIPPER_ERR_OPERATION_FAILED;
}

std::string fault_code_to_text_cpp(uint8_t fault_code)
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

void set_error(gripper_handle* handle,
               gripper_error_code_t code,
               const std::string& err)
{
    if (handle != nullptr)
    {
        handle->last_error_code = code;
        handle->last_error = err;
    }
}

void clear_error(gripper_handle* handle)
{
    if (handle != nullptr)
    {
        handle->last_error_code = GRIPPER_OK;
        handle->last_error.clear();
    }
}

gripper_error_code_t set_error_from_device(gripper_handle* handle)
{
    if (handle == nullptr)
    {
        return GRIPPER_ERR_INVALID_ARGUMENT;
    }

    const std::string err = handle->device.lastError();
    const gripper_error_code_t code = map_error_to_code(err);
    handle->last_error_code = code;
    handle->last_error = err;
    return code;
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
        return GRIPPER_ERR_INVALID_ARGUMENT;
    }

    if (!handle->device.connect())
    {
        return static_cast<int>(set_error_from_device(handle));
    }

    clear_error(handle);
    return GRIPPER_OK;
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
        return GRIPPER_ERR_INVALID_ARGUMENT;
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
        return static_cast<int>(set_error_from_device(handle));
    }

    if (out_result != nullptr)
    {
        copy_initialize_result(cpp_result, out_result);
    }

    clear_error(handle);
    return GRIPPER_OK;
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
        return GRIPPER_ERR_INVALID_ARGUMENT;
    }

    if (!handle->device.moveToOpeningMm(opening_mm))
    {
        return static_cast<int>(set_error_from_device(handle));
    }

    clear_error(handle);
    return GRIPPER_OK;
}

int gripper_move_to_opening_mm_with_limits(gripper_handle_t* handle,
                                           float opening_mm,
                                           float max_speed_mm_s,
                                           float max_current_amp)
{
    if (handle == nullptr)
    {
        return GRIPPER_ERR_INVALID_ARGUMENT;
    }

    if (!handle->device.moveToOpeningMmWithLimits(opening_mm,
                                                  max_speed_mm_s,
                                                  max_current_amp))
    {
        return static_cast<int>(set_error_from_device(handle));
    }

    clear_error(handle);
    return GRIPPER_OK;
}

int gripper_open(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return GRIPPER_ERR_INVALID_ARGUMENT;
    }

    if (!handle->device.open())
    {
        return static_cast<int>(set_error_from_device(handle));
    }

    clear_error(handle);
    return GRIPPER_OK;
}

int gripper_close(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return GRIPPER_ERR_INVALID_ARGUMENT;
    }

    if (!handle->device.close())
    {
        return static_cast<int>(set_error_from_device(handle));
    }

    clear_error(handle);
    return GRIPPER_OK;
}

int gripper_stop(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return GRIPPER_ERR_INVALID_ARGUMENT;
    }

    if (!handle->device.stop())
    {
        return static_cast<int>(set_error_from_device(handle));
    }

    clear_error(handle);
    return GRIPPER_OK;
}

int gripper_read_status(gripper_handle_t* handle, gripper_status_t* out_status)
{
    if (handle == nullptr || out_status == nullptr)
    {
        return GRIPPER_ERR_INVALID_ARGUMENT;
    }

    gripper::GripperStatus status{};
    if (!handle->device.readStatus(status))
    {
        return static_cast<int>(set_error_from_device(handle));
    }

    copy_status(status, out_status);
    clear_error(handle);
    return GRIPPER_OK;
}

int gripper_reboot(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return GRIPPER_ERR_INVALID_ARGUMENT;
    }

    if (!handle->device.reboot())
    {
        return static_cast<int>(set_error_from_device(handle));
    }

    clear_error(handle);
    return GRIPPER_OK;
}

int gripper_set_communication_config(gripper_handle_t* handle,
                                     uint8_t new_device_address,
                                     int new_baudrate)
{
    if (handle == nullptr)
    {
        return GRIPPER_ERR_INVALID_ARGUMENT;
    }

    if (!handle->device.setCommunicationConfig(new_device_address, new_baudrate))
    {
        return static_cast<int>(set_error_from_device(handle));
    }

    clear_error(handle);
    return GRIPPER_OK;
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

gripper_error_code_t gripper_get_last_error_code(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return GRIPPER_ERR_INVALID_ARGUMENT;
    }

    return handle->last_error_code;
}

const char* gripper_get_last_error(gripper_handle_t* handle)
{
    if (handle == nullptr)
    {
        return "invalid handle";
    }

    return handle->last_error.c_str();
}

const char* gripper_error_code_to_string(gripper_error_code_t code)
{
    switch (code)
    {
    case GRIPPER_OK:
        return "ok";
    case GRIPPER_ERR_INVALID_ARGUMENT:
        return "invalid argument";
    case GRIPPER_ERR_NOT_CONNECTED:
        return "device not connected";
    case GRIPPER_ERR_NOT_INITIALIZED:
        return "gripper not initialized";
    case GRIPPER_ERR_TIMEOUT:
        return "timeout";
    case GRIPPER_ERR_TRANSPORT:
        return "transport error";
    case GRIPPER_ERR_PROTOCOL:
        return "protocol error";
    case GRIPPER_ERR_BAD_RESPONSE:
        return "unexpected response";
    case GRIPPER_ERR_OUT_OF_RANGE:
        return "out of range";
    case GRIPPER_ERR_INVALID_CONFIG:
        return "invalid config";
    case GRIPPER_ERR_INVALID_STATE:
        return "invalid state";
    case GRIPPER_ERR_DEVICE_FAULT:
        return "device fault";
    case GRIPPER_ERR_OPERATION_FAILED:
        return "operation failed";
    case GRIPPER_ERR_COMM_CONFIG_APPLIED:
        return "communication config may have been applied";
    default:
        return "unknown error";
    }
}

const char* gripper_fault_code_to_string(uint8_t fault_code)
{
    static thread_local std::string text;
    text = fault_code_to_text_cpp(fault_code);
    return text.c_str();
}

int gripper_fault_code_has_voltage_fault(uint8_t fault_code)
{
    return ((fault_code & (1u << 0)) != 0u) ? 1 : 0;
}

int gripper_fault_code_has_current_fault(uint8_t fault_code)
{
    return ((fault_code & (1u << 1)) != 0u) ? 1 : 0;
}

int gripper_fault_code_has_temperature_fault(uint8_t fault_code)
{
    return ((fault_code & (1u << 2)) != 0u) ? 1 : 0;
}

int gripper_fault_code_has_encoder_fault(uint8_t fault_code)
{
    return ((fault_code & (1u << 3)) != 0u) ? 1 : 0;
}

int gripper_fault_code_has_hardware_fault(uint8_t fault_code)
{
    return ((fault_code & (1u << 6)) != 0u) ? 1 : 0;
}

int gripper_fault_code_has_software_fault(uint8_t fault_code)
{
    return ((fault_code & (1u << 7)) != 0u) ? 1 : 0;
}
} // extern "C"
