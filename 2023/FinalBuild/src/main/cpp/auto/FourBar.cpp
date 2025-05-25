#include "auto/FourBar.h"

RobotAutoMech::RobotAutoMech(FourBarArm& fourbararm)
: ArmControl{fourbararm}
{}

void RobotAutoMech::AutoInit()
{
    ArmControl.AutoInit();
}

int RobotAutoMech::AutoRun()
{
    return ArmControl.Auto();
}