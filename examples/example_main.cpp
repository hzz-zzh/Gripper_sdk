#include <iostream>
#include "gripper_sdk.h"

int main()
{
    gripper::Gripper g;

    if (!g.connect("COM3", 115200))
    {
        std::cout << "connect failed\n";
        return -1;
    }

    if (!g.enable())
    {
        std::cout << "enable failed\n";
        return -1;
    }

    if (!g.move(50.0f))
    {
        std::cout << "move failed\n";
        return -1;
    }

    std::cout << g.getStatus() << std::endl;

    g.disconnect();
    return 0;
}