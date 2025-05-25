#ifndef AUTO_WHEEL_ALIGN_H
#define AUTO_WHEEL_ALIGN_H

#include "autoBase.h"
#include "Drivetrain.h"

class RobotDriveAlign : public RobotAuto
{
public:
    RobotDriveAlign(Drivetrain &swervedrive)
    : Swervedrive{swervedrive}
    {}

    void AutoInit() override
    {
        autoTimer.Start();
    }

    int AutoRun() override
    {
        Swervedrive.AlignAllMotors();
        if(autoTimer.Get() > 1_s)
        {
            return 0;
        }
        return -1;
    }
private:

    Drivetrain &Swervedrive;
};

#endif