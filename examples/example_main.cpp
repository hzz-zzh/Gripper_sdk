#include "core/gripper.h"
#include <iomanip>
#include <iostream>

int main()
{
    gripper::Gripper g(0x01);

    if (!g.connect("/dev/ttyACM0", 115200))
    {
        std::cerr << "connect failed: " << g.lastError() << '\n';
        return 1;
    }

    gripper::VersionInfo version{};
    if (g.readVersion(version))
    {
        std::cout << "boot version: " << version.boot_version << '\n';
        std::cout << "app version : " << version.app_version << '\n';
        std::cout << "hw model    : " << version.hardware_model << '\n';
    }
    else
    {
        std::cerr << "readVersion failed: " << g.lastError() << '\n';
    }

    gripper::RealtimeStatus status{};
    if (g.readRealtime(status))
    {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "single turn deg : " << status.single_turn_deg << '\n';
        std::cout << "multi turn deg  : " << status.multi_turn_deg << '\n';
        std::cout << "speed rpm       : " << status.speed_rpm << '\n';
        std::cout << "q current amp   : " << status.q_current_amp << '\n';
        std::cout << "bus voltage v   : " << status.bus_voltage_v << '\n';
        std::cout << "bus current a   : " << status.bus_current_a << '\n';
        std::cout << "temperature c   : " << static_cast<int>(status.temperature_c) << '\n';
        std::cout << "run state       : " << static_cast<int>(status.run_state) << '\n';
        std::cout << "enabled         : " << (status.motor_enabled ? "yes" : "no") << '\n';
        std::cout << "fault code      : 0x" << std::hex << static_cast<int>(status.fault_code) << std::dec << '\n';
    }
    else
    {
        std::cerr << "readRealtime failed: " << g.lastError() << '\n';
    }

    // 示例：速度控制 100rpm，加速度 0 表示最大加速度
    // g.setSpeed(100.0f, 0, &status);

    // 示例：绝对位置到 16384 count（一圈）
    // g.moveToCount(16384, &status);

    // 示例：相对位置 +4096 count（90度）
    // g.moveByCount(4096, &status);

    g.disconnect();
    return 0;
}