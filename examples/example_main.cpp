#include "core/gripper_device.h"
#include <iostream>

int main()
{
    gripper::GripperDeviceConfig cfg;
    cfg.port_name = "/dev/ttyACM0";
    cfg.baudrate = 115200;
    cfg.device_address = 0x01;
    cfg.fully_close_count = 0;
    cfg.fully_open_count = 16384;

    gripper::GripperDevice g(cfg);

    if (!g.connect())
    {
        std::cerr << "connect failed: " << g.lastError() << '\n';
        return 1;
    }

    gripper::RealtimeStatus status{};

    if (!g.open(&status))
    {
        std::cerr << "open failed: " << g.lastError() << '\n';
        return 1;
    }

    if (!g.moveToPercent(50.0f, &status))
    {
        std::cerr << "moveToPercent failed: " << g.lastError() << '\n';
        return 1;
    }

    if (!g.close(&status))
    {
        std::cerr << "close failed: " << g.lastError() << '\n';
        return 1;
    }

    g.disconnect();
    return 0;
}