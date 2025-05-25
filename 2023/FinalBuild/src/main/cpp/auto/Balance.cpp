#include "auto/Balance.h"
#include <frc/smartdashboard/SmartDashboard.h>

    RobotAutoBalance::RobotAutoBalance( units::velocity::meters_per_second_t onGroundSpeed, 
                                        units::velocity::meters_per_second_t climbingSpeed, 
                                        RobotDirection Direction, 
                                        Drivetrain &swervedrive ) 
    :  Swervedrive{swervedrive}
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

    void RobotAutoBalance::AutoInit()
    {
        StartingAngle = fabs(RobotRoll().value());
        StartingAngleLower = StartingAngle - 3;
        StartingAngleUpper = StartingAngle + 3;

        // frc::SmartDashboard::PutNumber("Lower angle", StartingAngleLower);
        // frc::SmartDashboard::PutNumber("Upper angle", StartingAngleUpper);

        currState = APPROACHRAMP;
    }

    int RobotAutoBalance::AutoRun()
    {
        auto rollAbs = fabs(RobotRoll().value());

        switch (currState)
        {
     
        case APPROACHRAMP:
            Swervedrive.autoSwerve(GroundSpeed, 0_mps, 0_rad_per_s);
            if(RobotIsTilted())
            {
                currState = GOINGOVER;
            }
            break;
        
        case GOINGOVER: //chexks to see if Robot is at 11 or -11 degree meaning the robot is completely or mostly on charging station
            if(StartingAngleLower >= rollAbs || rollAbs >= StartingAngleUpper )
            {
                autoTimer.Start();
            }

            if(autoTimer.HasElapsed(1_s))
            {
                autoTimer.Stop();
                autoTimer.Reset();
                currState = ATTEMPTINGTOBALANCE;
            }
            break;

        case ATTEMPTINGTOBALANCE:
            if( StartingAngleLower < rollAbs && rollAbs < StartingAngleUpper )
            {
               Swervedrive.autoSwerve(0_mps, 0_mps, 0_rad_per_s);
               autoTimer.Start();
            }
            else
            {
                auto speed = (climbSpeed * (1 - (rollAbs / 16.25)));
                
                if((RobotRoll().value() < 0) && (rollAbs > 10))
                {
                    currState = CLIMBNG;
                    //MotionTimer.Reset();
                    autoTimer.Stop();
                    autoTimer.Reset();
                }
                
                Swervedrive.autoSwerve(speed, 0_mps, 0_rad_per_s);
                autoTimer.Stop();
                autoTimer.Reset();
            }
        
        break;

        case CLIMBNG:
        {
            MotionTimer.Start();
            auto StateOfMotion = fwdProfile.Calculate(MotionTimer.Get());
            Swervedrive.autoSwerve( -StateOfMotion.velocity, 0_mps, 0_rad_per_s);

            if(fwdProfile.IsFinished(MotionTimer.Get()))
            {
                currState = BRAKING;
                autoTimer.Reset();

                MotionTimer.Stop();
            }
        }
        break;

      /*  case CHECKBALANCE:
        {
            if(StartingAngleLower < rollAbs && rollAbs < StartingAngleUpper )
            {
                autoTimer.Reset();
                currState = BRAKING;  
            }
            else
            {
                currState = ATTEMPTINGTOBALANCE;
                
            }
        }
        break; */

        case BRAKING:
                Swervedrive.ParkingBrake();
                if(autoTimer.HasElapsed(1_s))
                {
                    currState = DONE;
                }
            break;

        case DONE:
                return 0;
            break;
        
        default:
            break;
        }
        return -1;
    }
