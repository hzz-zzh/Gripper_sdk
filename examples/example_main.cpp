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

    gripper::RealtimeStatus status{};
    if (!g.readRealtime(status))
    {
        std::cerr << "readRealtime failed: " << g.lastError() << '\n';
        return 1;
    }

    std::cout << "before calib: fault = 0x"
              << std::hex << static_cast<int>(status.fault_code) << std::dec << '\n';

    // 注意：校准时必须空载，且不要干扰电机转动

    if (!g.startEncoderCalibration(&status))
    {
        std::cerr << "startEncoderCalibration failed: " << g.lastError() << '\n';
        return 1;
    }

    std::cout << "calibration command sent, run_state = "
              << static_cast<int>(status.run_state) << '\n';


    // 如需中止校准，可发送 0x2F
    /*
    if (!g.motorOff(&status))
    {
        std::cerr << "motorOff failed: " << g.lastError() << '\n';
        return 1;
    }
    */

    g.disconnect();
    return 0;
}