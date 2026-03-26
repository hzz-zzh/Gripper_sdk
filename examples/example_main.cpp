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

    gripper::MotionControlParameters p{};
    if (!g.readMotionControlParameters(p))
    {
        std::cerr << "readMotionControlParameters failed: " << g.lastError() << '\n';
        return 1;
    }

    std::cout << "position_kp           = " << p.position_kp << '\n';
    std::cout << "position_ki           = " << p.position_ki << '\n';
    std::cout << "position_output_limit = " << p.position_output_limit
              << " (0.01 rpm)\n";
    std::cout << "speed_kp              = " << p.speed_kp << '\n';
    std::cout << "speed_ki              = " << p.speed_ki << '\n';
    std::cout << "speed_output_limit    = " << p.speed_output_limit
              << " (0.001 A)\n";

    // 最安全的写测试：原值写回，不保存
    /*
    gripper::MotionControlParameters after{};
    if (!g.writeMotionControlParametersTemp(p, &after))
    {
        std::cerr << "writeMotionControlParametersTemp failed: " << g.lastError() << '\n';
    }
    else
    {
        std::cout << "writeMotionControlParametersTemp ok\n";
    }
    */

    g.disconnect();
    return 0;
}