#ifndef AUTO_FOUR_BAR_H
#define AUTO_FOUR_BAR_H

#include "autoBase.h"
#include "FourBarArm.h"

class RobotAutoMech : public RobotAuto
{
public:
    RobotAutoMech(FourBarArm& fourbararm);
    void AutoInit() override;
    int AutoRun() override;

private:
    FourBarArm &ArmControl;
};

#endif