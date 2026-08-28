#include "c_api/gripper_c_api.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

static volatile sig_atomic_t stop_requested = 0;

static void handle_stop_signal(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static void print_status(const gripper_status_t* st)
{
    if (st == NULL)
    {
        return;
    }

    printf("opening=%.3f mm, opening_speed=%.3f mm/s, "
           "q_current=%.3f A, bus_voltage=%.3f V, bus_current=%.3f A, "
           "temp=%u C, run_state=%u, enabled=%d, fault=0x%02X (%s)\n",
           st->opening_mm,
           st->opening_speed_mm_s,
           st->q_current_amp,
           st->bus_voltage_v,
           st->bus_current_a,
           st->temperature_c,
           st->run_state,
           st->motor_enabled,
           st->fault_code,
           gripper_fault_code_to_string(st->fault_code));
}

static void print_api_error(gripper_handle_t* h, const char* action, int rc)
{
    const char* code_text = gripper_error_code_to_string((gripper_error_code_t)rc);
    const char* detail = gripper_get_last_error(h);

    if (code_text == NULL)
    {
        code_text = "unknown";
    }
    if (detail == NULL)
    {
        detail = "";
    }

    printf("%s failed: rc=%d (%s), detail=%s\n", action, rc, code_text, detail);
}

int main(void)
{
    signal(SIGINT, handle_stop_signal);
    signal(SIGTERM, handle_stop_signal);

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

    int rc = gripper_connect(h);
    if (rc != GRIPPER_OK)
    {
        print_api_error(h, "connect", rc);
        gripper_destroy(h);
        return 1;
    }

    printf("connect ok\n");
    printf("opening range: [%.3f, %.3f] mm\n",
           gripper_get_min_opening_mm(h),
           gripper_get_max_opening_mm(h));

    gripper_status_t before;
    rc = gripper_read_status(h, &before);
    if (rc == GRIPPER_OK)
    {
        printf("before homing:\n");
        print_status(&before);
    }
    else
    {
        print_api_error(h, "read_status before homing", rc);
    }

    gripper_initialize_config_t hc;
    gripper_initialize_result_t hr;

    gripper_initialize_config_init(&hc);

    hc.search_speed_mm_s = 50.0f;
    hc.current_limit_amp = 1.8f;
    hc.search_direction = +1;
    hc.poll_interval_ms = 20;
    hc.timeout_ms = 5000;
    hc.speed_epsilon_mm_s = 0.25f;
    hc.current_threshold_a = 1.3f;
    hc.position_epsilon_mm = 0.05f;
    hc.detect_consecutive_samples = 4;
    hc.clear_fault_before_start = 1;
    hc.set_zero_after_detect = 1;
    hc.backoff_after_zero_mm = 2.0f;
    hc.open_safety_margin_mm = 8.0f;

    printf("start homing with current limit %.3f A...\n", hc.current_limit_amp);
    rc = gripper_initialize(h, &hc, &hr);
    if (rc != GRIPPER_OK)
    {
        print_api_error(h, "homing", rc);
        gripper_stop(h);
        gripper_disconnect(h);
        gripper_destroy(h);
        return 1;
    }

    printf("homing ok\n");
    printf("  current_limit_during_homing  = %.3f A\n",
           hr.current_limit_amp_during_homing);
    printf("  limit_detected               = %d\n", hr.limit_detected);
    printf("  zero_set                     = %d\n", hr.zero_set);
    printf("  backoff_done                 = %d\n", hr.backoff_done);
    printf("  detect_samples               = %d\n", hr.detect_samples);
    printf("  limit_opening_before_zero    = %.3f mm\n", hr.limit_opening_mm_before_zero);
    printf("  mechanical_offset            = %u\n", hr.mechanical_offset);
    printf("final status after homing:\n");
    print_status(&hr.final_status);

    while (!stop_requested)
    {
        rc = gripper_move_to_opening_mm_with_limits(h, 0.0f, 150.0f, 1.5f);
        if (rc != GRIPPER_OK)
        {
            print_api_error(h, "move to 0.0 mm", rc);
            break;
        }

        usleep(1000 * 1000);
        if (stop_requested)
        {
            break;
        }

        gripper_status_t st;
        rc = gripper_read_status(h, &st);
        if (rc == GRIPPER_OK)
        {
            print_status(&st);
        }
        else
        {
            print_api_error(h, "read_status in loop", rc);
            break;
        }

        rc = gripper_move_to_opening_mm_with_limits(h, 105.0f, 150.0f, 1.5f);
        if (rc != GRIPPER_OK)
        {
            print_api_error(h, "move to 105.0 mm", rc);
            break;
        }

        usleep(1000 * 1000);
        if (stop_requested)
        {
            break;
        }

        rc = gripper_read_status(h, &st);
        if (rc == GRIPPER_OK)
        {
            print_status(&st);
        }
        else
        {
            print_api_error(h, "read_status in loop", rc);
            break;
        }
    }

    if (stop_requested)
    {
        printf("stop requested, disabling motor...\n");
    }

    rc = gripper_stop(h);
    if (rc != GRIPPER_OK)
    {
        print_api_error(h, "stop", rc);
    }

    gripper_disconnect(h);
    gripper_destroy(h);
    return 0;
}
