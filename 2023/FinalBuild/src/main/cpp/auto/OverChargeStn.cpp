#include "auto/OverChargeStn.h"

RobotOverChargeStation::RobotOverChargeStation(units::velocity::meters_per_second_t onGroundSpeed, units::velocity::meters_per_second_t climbingSpeed, RobotDirection Direction, Drivetrain &swervedrive)
    : SwerveDrive{swervedrive}
{
    if(Direction == AwayFromOpposingTeam)
    {
        GroundSpeed = -onGroundSpeed;
        climbSpeed = -climbingSpeed;

        First_Tilt = AWAY_OPPOSING_TILT;
        Last_Tilt = TOWARD_OPPOSING_TILT;
    }
    else
    {
        GroundSpeed = onGroundSpeed;
        climbSpeed = climbingSpeed;

        First_Tilt = TOWARD_OPPOSING_TILT;
        Last_Tilt = AWAY_OPPOSING_TILT;
    }
}

void RobotOverChargeStation::AutoInit()
{
    currState = APPROACHRAMP;
}

int RobotOverChargeStation::AutoRun()
{
    switch (currState)
    {
    case APPROACHRAMP:

        SwerveDrive.autoSwerve(GroundSpeed, 0_mps, 0_rad_per_s);
        if(RobotIsTilted())
        {
            currState = GOINGOVER;
        }
    break;
    
    case GOINGOVER:

        SwerveDrive.autoSwerve(climbSpeed, 0_mps, 0_rad_per_s);
        if(RobotIsLevel())
        {
            autoTimer.Start();
        }
        else
        {
            autoTimer.Stop();
            autoTimer.Reset();
        }

        if(autoTimer.HasElapsed(1_s))
        {     
            currState = ONGROUND;
        }
    break;

    case ONGROUND:
        SwerveDrive.autoSwerve(-GroundSpeed, 0_mps, 0_rad_per_s);
        if(RobotIsTilted())
        {
            currState = RETURNBALANCE;
        }
    break;

    case RETURNBALANCE:
        {
            MotionTimer.Start();
            auto StateOfMotion = fwdProfile.Calculate(MotionTimer.Get());
            SwerveDrive.autoSwerve( -StateOfMotion.velocity, 0_mps, 0_rad_per_s);

            if(fwdProfile.IsFinished(MotionTimer.Get()))
            {
                currState = BRAKING;


                MotionTimer.Stop();
            }
        }
        break;
    
    case BRAKING:
        SwerveDrive.ParkingBrake();
        if(autoTimer.HasElapsed(1_s))
        {
            currState = DONE;
        }
        break;

    case DONE:
        return 0;
    break;

    default:
    return -2;
    break;
    }

    return -1;
}
