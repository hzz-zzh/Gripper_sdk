#include "c_api/gripper_c_api.h"
#include <stdio.h>

static void print_status(const gripper_realtime_status_t* st)
{
    if (st == NULL)
    {
        return;
    }

    printf("count=%d, speed=%.3f rpm, q_current=%.3f A, voltage=%.3f V, fault=0x%02X\n",
           st->multi_turn_count,
           st->speed_rpm,
           st->q_current_amp,
           st->bus_voltage_v,
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

    gripper_realtime_status_t before;
    if (gripper_read_realtime(h, &before) == GRIPPER_API_OK)
    {
        printf("before homing:\n");
        print_status(&before);
    }
    else
    {
        printf("read_realtime before homing failed: %s\n", gripper_get_last_error(h));
    }

    gripper_initialize_config_t hc;
    gripper_initialize_result_t hr;

    gripper_initialize_config_init(&hc);

    /* 下面这些参数需要根据你的机构慢慢调 */
    hc.search_speed_rpm = 100.0f;
    hc.search_direction = +1;          /* 如果方向反了就改成 -1 */
    hc.poll_interval_ms = 20;
    hc.timeout_ms = 5000;
    hc.speed_epsilon_rpm = 0.5f;
    hc.current_threshold_a = 0.5f;
    hc.position_epsilon_count = 5;
    hc.detect_consecutive_samples = 4;
    hc.clear_fault_before_start = 1;
    hc.set_zero_after_detect = 1;
    hc.backoff_count_after_zero = -15000;

    printf("start homing...\n");
    if (gripper_initialize(h, &hc, &hr) != GRIPPER_API_OK)
    {
        printf("homing failed: %s\n", gripper_get_last_error(h));
        gripper_disconnect(h);
        gripper_destroy(h);
        return 1;
    }

    printf("homing ok\n");

    // /* Homing 完成后，做一个小动作验证 */
    // if (gripper_move_to_percent(h, 10.0f) != GRIPPER_API_OK)
    // {
    //     printf("move_to_percent(10) failed: %s\n", gripper_get_last_error(h));
    // }
    // else
    // {
    //     gripper_realtime_status_t after_move;
    //     if (gripper_read_realtime(h, &after_move) == GRIPPER_API_OK)
    //     {
    //         printf("after move_to_percent(10):\n");
    //         print_status(&after_move);
    //     }
    //     else
    //     {
    //         printf("read_realtime after move failed: %s\n", gripper_get_last_error(h));
    //     }
    // }

    while(1){

        gripper_move_to_position_with_limits(h,
                                     -40000,
                                     200.0f,   // 最大速度 80 rpm
                                     2.0f);   // 最大电流 1.5 A
        // gripper_move_to_position(h,-40000);
        // gripper_move_to_percent(h, 80.0f);
        // gripper_close(h);
        // gripper_open(h);
        // gripper_stop(h);
    }

    gripper_stop(h);
    gripper_disconnect(h);
    gripper_destroy(h);

    return 0;
}