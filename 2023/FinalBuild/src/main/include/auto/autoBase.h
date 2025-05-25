#ifndef ROBOT_BASE_CLASS_H
#define ROBOT_BASE_CLASS_H

#include <frc/Timer.h>

class RobotAuto
{
public:
    virtual ~RobotAuto() = default;
    virtual void AutoInit() = 0;
    virtual int AutoRun() = 0;

protected:
    frc::Timer autoTimer;
};

#endif