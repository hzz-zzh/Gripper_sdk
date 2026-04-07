#include "c_api/gripper_c_api.h"

#include <stdint.h>
#include <stdio.h>

static void print_status(const gripper_status_t* st)
{
    if (st == NULL)
    {
        return;
    }

    printf("opening=%.3f mm, opening_speed=%.3f mm/s, q_current=%.3f A, "
           "bus_voltage=%.3f V, bus_current=%.3f A, temp=%u C, run_state=%u, enabled=%d, fault=0x%02X\n",
           st->opening_mm,
           st->opening_speed_mm_s,
           st->q_current_amp,
           st->bus_voltage_v,
           st->bus_current_a,
           st->temperature_c,
           st->run_state,
           st->motor_enabled,
           st->fault_code);
}

static int connect_or_print_error(gripper_handle_t* h, const char* tag)
{
    const int ret = gripper_connect(h);
    if (ret != GRIPPER_API_OK)
    {
        printf("[%s] connect failed: %s\n", tag, gripper_get_last_error(h));
        return 0;
    }

    printf("[%s] connect ok\n", tag);
    return 1;
}

int main(void)
{
    const char* port_name = "/dev/ttyACM0";
    const int old_baudrate = 115200;
    const uint8_t old_device_address = 0x01;

    const int new_baudrate = 460800;
    const uint8_t new_device_address = 0x02;

    printf("=== gripper communication config demo ===\n");
    printf("port             : %s\n", port_name);
    printf("old config       : address=0x%02X, baudrate=%d\n", old_device_address, old_baudrate);
    printf("target config    : address=0x%02X, baudrate=%d\n", new_device_address, new_baudrate);

    if (new_device_address == 0 || new_device_address == 0xFF)
    {
        printf("invalid new_device_address, expected 1~254\n");
        return 1;
    }

    gripper_config_t cfg;
    cfg.port_name = port_name;
    cfg.baudrate = old_baudrate;
    cfg.device_address = old_device_address;
    cfg.timeout_ms = 300;

    gripper_handle_t* h = gripper_create(&cfg);
    if (h == NULL)
    {
        printf("gripper_create failed\n");
        return 1;
    }

    if (!connect_or_print_error(h, "step1-old-config"))
    {
        gripper_destroy(h);
        return 1;
    }

    gripper_status_t st_before;
    if (gripper_read_status(h, &st_before) == GRIPPER_API_OK)
    {
        printf("status before change:\n");
        print_status(&st_before);
    }
    else
    {
        printf("read_status(before) failed: %s\n", gripper_get_last_error(h));
    }

    printf("\nwriting new communication config only (no reboot in API) ...\n");
    if (gripper_set_communication_config(h, new_device_address, new_baudrate) != GRIPPER_API_OK)
    {
        printf("set communication config failed: %s\n", gripper_get_last_error(h));
        gripper_disconnect(h);
        gripper_destroy(h);
        return 1;
    }

    printf("set communication config ok\n");

    printf("set communication config ok\n");
    printf("new address / baudrate have been saved to device parameters.\n");
    printf("they will take effect only after device reboot.\n");

    gripper_status_t st_after_write;
    if (gripper_read_status(h, &st_after_write) == GRIPPER_API_OK)
    {
        printf("status after write (current session still uses old config):\n");
        print_status(&st_after_write);
    }
    else
    {
        printf("read_status(after write) failed: %s\n", gripper_get_last_error(h));
    }

    printf("\nNEXT STEP:\n");
    printf("1) reboot device manually, or call gripper_reboot() yourself when you want.\n");
    printf("2) disconnect current handle.\n");
    printf("3) recreate handle with new address=0x%02X and baudrate=%d, then reconnect.\n",
           new_device_address,
           new_baudrate);

    gripper_reboot(h);

    gripper_disconnect(h);
    gripper_destroy(h);
    return 0;
}
