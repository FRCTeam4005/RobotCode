#ifndef AUTO_BALANCE
#define AUTO_BALANCE

#include "autoBase.h"
#include "LevelBase.h"
#include "Drivetrain.h"
#include <frc/trajectory/TrapezoidProfile.h>

class RobotAutoBalance : public RobotAuto, public RobotLeveling
{
public:
    RobotAutoBalance( units::velocity::meters_per_second_t onGroundSpeed, 
                      units::velocity::meters_per_second_t climbingSpeed, 
                      RobotDirection Direction, 
                      Drivetrain &swervedrive);
    void AutoInit() override;
    int AutoRun() override;

private:
    typedef enum
    {
        APPROACHRAMP,
        GOINGOVER,
        ATTEMPTINGTOBALANCE,
        CHECKBALANCE,
        CLIMBNG,
        BRAKING,
        DONE,
    }BalancingStates_t;

    BalancingStates_t currState;
    Drivetrain &Swervedrive;

    frc::Timer MotionTimer{};
    
    frc::TrapezoidProfile<units::meters>::Constraints fwdConstraints{1_fps, 1_mps_sq};
    frc::TrapezoidProfile<units::meters>::State fwdStartState{0_m, 0_mps};
    frc::TrapezoidProfile<units::meters>::State fwdEndState{13.75_in, 0_mps};
    frc::TrapezoidProfile<units::meters> fwdProfile{fwdConstraints, fwdEndState, fwdStartState};

    
};

#endif