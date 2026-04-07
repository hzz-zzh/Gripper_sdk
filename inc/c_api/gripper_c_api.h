#ifndef GRIPPER_C_API_H
#define GRIPPER_C_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gripper_handle gripper_handle_t;

typedef struct
{
    const char* port_name;
    int baudrate;
    uint8_t device_address;
    int timeout_ms;
} gripper_config_t;

typedef struct
{
    float opening_mm;
    float opening_speed_mm_s;
    float q_current_amp;
    float bus_voltage_v;
    float bus_current_a;
    uint8_t temperature_c;
    uint8_t run_state;
    int motor_enabled;
    uint8_t fault_code;
} gripper_status_t;

typedef struct
{
    float search_speed_mm_s;
    int search_direction;

    int poll_interval_ms;
    int timeout_ms;

    float speed_epsilon_mm_s;
    float current_threshold_a;
    float position_epsilon_mm;
    int detect_consecutive_samples;

    int clear_fault_before_start;
    int set_zero_after_detect;
    float backoff_after_zero_mm;
} gripper_initialize_config_t;

typedef struct
{
    int limit_detected;
    int zero_set;
    int backoff_done;

    int detect_samples;
    float limit_opening_mm_before_zero;
    uint16_t mechanical_offset;

    gripper_status_t final_status;
} gripper_initialize_result_t;

typedef enum
{
    GRIPPER_OK = 0,

    GRIPPER_ERR_INVALID_ARGUMENT = -1,
    GRIPPER_ERR_NOT_CONNECTED = -2,
    GRIPPER_ERR_NOT_INITIALIZED = -3,

    GRIPPER_ERR_TIMEOUT = -10,
    GRIPPER_ERR_TRANSPORT = -11,
    GRIPPER_ERR_PROTOCOL = -12,
    GRIPPER_ERR_BAD_RESPONSE = -13,

    GRIPPER_ERR_OUT_OF_RANGE = -20,
    GRIPPER_ERR_INVALID_CONFIG = -21,
    GRIPPER_ERR_INVALID_STATE = -22,

    GRIPPER_ERR_DEVICE_FAULT = -30,
    GRIPPER_ERR_OPERATION_FAILED = -31,

    GRIPPER_ERR_COMM_CONFIG_APPLIED = 1
} gripper_error_code_t;

/* 兼容旧接口习惯： */
#define GRIPPER_API_OK GRIPPER_OK
#define GRIPPER_API_ERROR GRIPPER_ERR_OPERATION_FAILED
#define GRIPPER_API_INVALID_ARGUMENT GRIPPER_ERR_INVALID_ARGUMENT

gripper_handle_t* gripper_create(const gripper_config_t* config);
void gripper_destroy(gripper_handle_t* handle);

int gripper_connect(gripper_handle_t* handle);
void gripper_disconnect(gripper_handle_t* handle);
int gripper_is_connected(gripper_handle_t* handle);

void gripper_initialize_config_init(gripper_initialize_config_t* config);
int gripper_initialize(gripper_handle_t* handle,
                       const gripper_initialize_config_t* config,
                       gripper_initialize_result_t* out_result);
int gripper_is_initialized(gripper_handle_t* handle);

int gripper_move_to_opening_mm(gripper_handle_t* handle, float opening_mm);
int gripper_move_to_opening_mm_with_limits(gripper_handle_t* handle,
                                           float opening_mm,
                                           float max_speed_mm_s,
                                           float max_current_amp);

int gripper_open(gripper_handle_t* handle);
int gripper_close(gripper_handle_t* handle);
int gripper_stop(gripper_handle_t* handle);

int gripper_read_status(gripper_handle_t* handle, gripper_status_t* out_status);
int gripper_reboot(gripper_handle_t* handle);

int gripper_set_communication_config(gripper_handle_t* handle,
                                     uint8_t new_device_address,
                                     int new_baudrate);

float gripper_get_min_opening_mm(gripper_handle_t* handle);
float gripper_get_max_opening_mm(gripper_handle_t* handle);

gripper_error_code_t gripper_get_last_error_code(gripper_handle_t* handle);
const char* gripper_get_last_error(gripper_handle_t* handle);
const char* gripper_error_code_to_string(gripper_error_code_t code);

const char* gripper_fault_code_to_string(uint8_t fault_code);
int gripper_fault_code_has_voltage_fault(uint8_t fault_code);
int gripper_fault_code_has_current_fault(uint8_t fault_code);
int gripper_fault_code_has_temperature_fault(uint8_t fault_code);
int gripper_fault_code_has_encoder_fault(uint8_t fault_code);
int gripper_fault_code_has_hardware_fault(uint8_t fault_code);
int gripper_fault_code_has_software_fault(uint8_t fault_code);

#ifdef __cplusplus
}
#endif

#endif
