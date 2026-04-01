#include "c_api/gripper_c_api.h"
#include <stdio.h>
#include <unistd.h>

static void print_status(const gripper_status_t* st)
{
    if (st == NULL)
    {
        return;
    }

    printf("opening=%.3f mm, opening_speed=%.3f mm/s, q_current=%.3f A, voltage=%.3f V, run_state=%u, enabled=%d, fault=0x%02X\n",
           st->opening_mm,
           st->opening_speed_mm_s,
           st->q_current_amp,
           st->bus_voltage_v,
           st->run_state,
           st->motor_enabled,
           st->fault_code);
}

int main(void)
{
    gripper_config_t cfg;
    cfg.port_name = "/dev/ttyACM0";
    cfg.baudrate = 115200;
    cfg.device_address = 0x01;
    cfg.timeout_ms = 300;

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

    printf("connect ok\n");
    printf("opening range: [%.3f, %.3f] mm\n",
           gripper_get_min_opening_mm(h),
           gripper_get_max_opening_mm(h));

    gripper_status_t before;
    if (gripper_read_status(h, &before) == GRIPPER_API_OK)
    {
        printf("before homing:\n");
        print_status(&before);
    }
    else
    {
        printf("read_status before homing failed: %s\n", gripper_get_last_error(h));
    }

    gripper_initialize_config_t hc;
    gripper_initialize_result_t hr;

    gripper_initialize_config_init(&hc);

    hc.search_speed_mm_s = 50.0f;
    hc.search_direction = +1;
    hc.poll_interval_ms = 20;
    hc.timeout_ms = 5000;
    hc.speed_epsilon_mm_s = 0.25f;
    hc.current_threshold_a = 0.5f;
    hc.position_epsilon_mm = 0.05f;
    hc.detect_consecutive_samples = 4;
    hc.clear_fault_before_start = 1;
    hc.set_zero_after_detect = 1;
    hc.backoff_after_zero_mm = 2.0f;

    printf("start homing...\n");
    if (gripper_initialize(h, &hc, &hr) != GRIPPER_API_OK)
    {
        printf("homing failed: %s\n", gripper_get_last_error(h));
        gripper_disconnect(h);
        gripper_destroy(h);
        return 1;
    }

    printf("homing ok, limit opening before zero = %.3f mm\n",
           hr.limit_opening_mm_before_zero);

               
    
    if (gripper_move_to_opening_mm_with_limits(h,
                                               20.0f,
                                               50.0f,
                                               2.0f) != GRIPPER_API_OK)
    {
        printf("move_to_opening_mm_with_limits failed: %s\n",
               gripper_get_last_error(h));
    }
    else
    {
        gripper_status_t after_move;
        if (gripper_read_status(h, &after_move) == GRIPPER_API_OK)
        {
            printf("after move:\n");
            print_status(&after_move);
        }
    }
    // usleep(4000 * 1000);



    // gripper_stop(h);
    gripper_disconnect(h);
    gripper_destroy(h);
    return 0;
}