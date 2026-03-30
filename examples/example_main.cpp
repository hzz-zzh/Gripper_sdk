#include "core/gripper_device.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>

namespace
{
void printMenu()
{
    std::cout << "\n========== GRIPPER DEVICE MENU ==========\n";
    std::cout << "1. Read realtime status\n";
    std::cout << "2. Open gripper\n";
    std::cout << "3. Close gripper\n";
    std::cout << "4. Move gripper to percent\n";
    std::cout << "5. Stop gripper\n";
    std::cout << "6. Reboot device\n";
    std::cout << "0. Exit\n";
    std::cout << "=========================================\n";
    std::cout << "Select: ";
}

void clearInput()
{
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void printStatus(const gripper::RealtimeStatus& s, const gripper::GripperDevice& dev)
{
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "single_turn_deg : " << s.single_turn_deg << '\n';
    std::cout << "multi_turn_deg  : " << s.multi_turn_deg << '\n';
    std::cout << "multi_turn_count: " << s.multi_turn_count << '\n';
    std::cout << "open_percent    : " << dev.countToPercent(s.multi_turn_count) << '\n';
    std::cout << "speed_rpm       : " << s.speed_rpm << '\n';
    std::cout << "q_current_amp   : " << s.q_current_amp << '\n';
    std::cout << "bus_voltage_v   : " << s.bus_voltage_v << '\n';
    std::cout << "bus_current_a   : " << s.bus_current_a << '\n';
    std::cout << "temperature_c   : " << static_cast<int>(s.temperature_c) << '\n';
    std::cout << "run_state       : " << static_cast<int>(s.run_state) << '\n';
    std::cout << "motor_enabled   : " << (s.motor_enabled ? "yes" : "no") << '\n';
    std::cout << "fault_code      : 0x"
              << std::hex << static_cast<int>(s.fault_code) << std::dec << '\n';
}
}

int main()
{
    gripper::GripperDeviceConfig cfg;
    cfg.port_name = "/dev/ttyACM0";
    cfg.baudrate = 115200;
    cfg.device_address = 0x01;
    cfg.timeout_ms = 300;

    gripper::GripperDevice dev(cfg);

    if (!dev.connect())
    {
        std::cerr << "connect failed: " << dev.lastError() << '\n';
        return 1;
    }

    std::cout << "connected to " << cfg.port_name
              << ", baudrate=" << cfg.baudrate
              << ", device=0x" << std::hex << static_cast<int>(cfg.device_address)
              << std::dec << "\n";

    bool running = true;
    while (running)
    {
        printMenu();

        int cmd = -1;
        if (!(std::cin >> cmd))
        {
            clearInput();
            std::cout << "invalid input\n";
            continue;
        }

        switch (cmd)
        {
        case 0:
            running = false;
            break;

        case 1:
        {
            gripper::RealtimeStatus s{};
            if (!dev.readRealtime(s))
            {
                std::cerr << "readRealtime failed: " << dev.lastError() << '\n';
                break;
            }

            printStatus(s, dev);
            break;
        }

        case 2:
        {
            gripper::RealtimeStatus s{};
            if (!dev.open(&s))
            {
                std::cerr << "open failed: " << dev.lastError() << '\n';
                break;
            }

            std::cout << "open ok\n";
            printStatus(s, dev);
            break;
        }

        case 3:
        {
            gripper::RealtimeStatus s{};
            if (!dev.close(&s))
            {
                std::cerr << "close failed: " << dev.lastError() << '\n';
                break;
            }

            std::cout << "close ok\n";
            printStatus(s, dev);
            break;
        }

        case 4:
        {
            float percent = 0.0f;
            std::cout << "input percent (0~100): ";
            if (!(std::cin >> percent))
            {
                clearInput();
                std::cout << "invalid input\n";
                break;
            }

            gripper::RealtimeStatus s{};
            if (!dev.moveToPercent(percent, &s))
            {
                std::cerr << "moveToPercent failed: " << dev.lastError() << '\n';
                break;
            }

            std::cout << "moveToPercent ok\n";
            printStatus(s, dev);
            break;
        }

        case 5:
        {
            gripper::RealtimeStatus s{};
            if (!dev.stop(&s))
            {
                std::cerr << "stop failed: " << dev.lastError() << '\n';
                break;
            }

            std::cout << "stop ok\n";
            printStatus(s, dev);
            break;
        }

        case 6:
        {
            char confirm = 'n';
            std::cout << "reboot device now? (y/N): ";
            std::cin >> confirm;

            if (confirm != 'y' && confirm != 'Y')
            {
                std::cout << "canceled\n";
                break;
            }

            if (!dev.motor().reboot())
            {
                std::cerr << "reboot failed: " << dev.motor().lastError() << '\n';
                break;
            }

            std::cout << "reboot command sent, waiting device restart...\n";
            dev.disconnect();
            std::this_thread::sleep_for(std::chrono::milliseconds(1200));

            if (!dev.connect())
            {
                std::cerr << "reconnect failed after reboot: " << dev.lastError() << '\n';
                break;
            }

            std::cout << "reconnected after reboot\n";
            break;
        }

        default:
            std::cout << "unknown command\n";
            break;
        }
    }

    dev.disconnect();
    return 0;
}