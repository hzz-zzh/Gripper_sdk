#include "c_api/gripper_c_api.h"
#include <stdio.h>

int main(void)
{
    gripper_config_t cfg;
    cfg.port_name = "/dev/ttyACM0";
    cfg.baudrate = 115200;
    cfg.device_address = 0x01;
    cfg.timeout_ms = 300;
    cfg.fully_open_count = 16384;
    cfg.fully_close_count = 0;
    cfg.home_count = 0;

    gripper_handle_t* h = gripper_create(&cfg);
    if (h == NULL)
    {
        printf("gripper_create failed\n");
        return 1;
    }

    if (gripper_connect(h) != GRIPPER_API_OK)
    {
        printf("connect failed: %s\n", gripper_get_last_error(h));
        gripper_destroy(h);
        return 1;
    }

    if (gripper_open(h) != GRIPPER_API_OK)
    {
        printf("open failed: %s\n", gripper_get_last_error(h));
        gripper_destroy(h);
        return 1;
    }

    if (gripper_move_to_percent(h, 50.0f) != GRIPPER_API_OK)
    {
        printf("move_to_percent failed: %s\n", gripper_get_last_error(h));
        gripper_destroy(h);
        return 1;
    }

    gripper_realtime_status_t st;
    if (gripper_read_realtime(h, &st) == GRIPPER_API_OK)
    {
        printf("count=%d, speed=%.3f rpm, voltage=%.3f V, fault=0x%02X\n",
               st.multi_turn_count,
               st.speed_rpm,
               st.bus_voltage_v,
               st.fault_code);
    }
    else
    {
        printf("read_realtime failed: %s\n", gripper_get_last_error(h));
    }

    gripper_stop(h);
    gripper_disconnect(h);
    gripper_destroy(h);
    return 0;
}