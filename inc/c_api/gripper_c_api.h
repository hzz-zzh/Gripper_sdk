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

enum
{
    GRIPPER_API_OK = 0,
    GRIPPER_API_ERROR = -1,
    GRIPPER_API_INVALID_ARGUMENT = -2
};

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

float gripper_get_min_opening_mm(gripper_handle_t* handle);
float gripper_get_max_opening_mm(gripper_handle_t* handle);

const char* gripper_get_last_error(gripper_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif