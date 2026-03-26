#ifndef GRIPPER_SDK_H
#define GRIPPER_SDK_H

#include <string>

namespace gripper
{
class Gripper
{
public:
    Gripper();

    bool connect(const std::string& port, int baudrate);
    void disconnect();

    bool enable();
    bool move(float position);
    std::string getStatus() const;

private:
    bool connected_;
    bool enabled_;
    float current_position_;
};
}

#endif
