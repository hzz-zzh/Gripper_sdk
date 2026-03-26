#include "core/gripper.h"
#include <iostream>

int main()
{
    gripper::Gripper g(0x01);

    if (!g.connect("/dev/ttyACM0", 115200))
    {
        std::cerr << "connect failed: " << g.lastError() << '\n';
        return 1;
    }

    gripper::MotorHardwareParameters p{};
    if (!g.readMotorHardwareParameters(p))
    {
        std::cerr << "readMotorHardwareParameters failed: " << g.lastError() << '\n';
        return 1;
    }

    std::cout << "motor_name             = " << p.motor_name << '\n';
    std::cout << "pole_pairs             = " << static_cast<int>(p.pole_pairs) << '\n';
    std::cout << "phase_resistance_ohm   = " << p.phase_resistance_ohm << '\n';
    std::cout << "phase_inductance_mh    = " << p.phase_inductance_mh << '\n';
    std::cout << "torque_constant_nm     = " << p.torque_constant_nm << '\n';
    std::cout << "gear_ratio             = " << static_cast<int>(p.gear_ratio) << '\n';

    /*
    // 最安全的写测试：原值写回
    gripper::MotorHardwareParameters after{};
    if (!g.writeMotorHardwareParameters(p, &after))
    {
        std::cerr << "writeMotorHardwareParameters failed: " << g.lastError() << '\n';
    }
    else
    {
        std::cout << "writeMotorHardwareParameters ok\n";
    }
    */

    g.disconnect();
    return 0;
}