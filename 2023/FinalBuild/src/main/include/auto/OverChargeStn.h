#ifndef AUTO_OVER_CHARGE_STATION
#define AUTO_OVER_CHARGE_STATION

#include "autoBase.h"
#include "LevelBase.h"
#include "Drivetrain.h"

#include <frc/trajectory/TrapezoidProfile.h>

class RobotOverChargeStation : public RobotAuto, public RobotLeveling
{
public:
    RobotOverChargeStation(units::velocity::meters_per_second_t onGroundSpeed, units::velocity::meters_per_second_t climbingSpeed, RobotDirection Direction, Drivetrain &swervedrive);
    void AutoInit() override;
    int AutoRun() override;

private:

    Drivetrain &SwerveDrive;

    typedef enum
    {
        APPROACHRAMP,
        GOINGOVER,
        ONGROUND,
        RETURNBALANCE,
        BRAKING,
        DONE
    }OverChargeStates_t;

    OverChargeStates_t currState;

        frc::Timer MotionTimer{};
    
    frc::TrapezoidProfile<units::meters>::Constraints fwdConstraints{1_fps, 1_mps_sq};
    frc::TrapezoidProfile<units::meters>::State fwdStartState{0_m, 0_mps};
    frc::TrapezoidProfile<units::meters>::State fwdEndState{32_in, 0_mps};
    frc::TrapezoidProfile<units::meters> fwdProfile{fwdConstraints, fwdEndState, fwdStartState};
};

#endif