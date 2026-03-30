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

    int32_t fully_open_count;
    int32_t fully_close_count;
    int32_t home_count;
} gripper_config_t;

typedef struct
{
    uint16_t single_turn_raw;
    float single_turn_deg;

    int32_t multi_turn_count;
    float multi_turn_deg;

    int32_t speed_raw;
    float speed_rpm;

    int32_t q_current_raw;
    float q_current_amp;

    uint16_t bus_voltage_raw;
    float bus_voltage_v;

    uint16_t bus_current_raw;
    float bus_current_a;

    uint8_t temperature_c;
    uint8_t run_state;
    int motor_enabled;
    uint8_t fault_code;
} gripper_realtime_status_t;

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

int gripper_open(gripper_handle_t* handle);
int gripper_close(gripper_handle_t* handle);
int gripper_move_to_percent(gripper_handle_t* handle, float percent);
int gripper_stop(gripper_handle_t* handle);

int gripper_read_realtime(gripper_handle_t* handle, gripper_realtime_status_t* out_status);

int gripper_reboot(gripper_handle_t* handle);

const char* gripper_get_last_error(gripper_handle_t* handle);

#ifdef __cplusplus
}
#endif

#endif