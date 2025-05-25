#ifndef AUTO_DRIVE
#define AUTO_DRIVE

#include "autoBase.h"
#include <units/length.h>
#include <units/math.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/filter/SlewRateLimiter.h>

#include "Drivetrain.h"
#include "RobotConstants.h"
#include <units/angular_acceleration.h>

class RobotAutoDrive : public RobotAuto
{
public:

    RobotAutoDrive(units::meter_t fwd, units::meter_t right, units::radian_t theta, units::meters_per_second_t speed, Drivetrain &swervedrive);
    void AutoInit() override;
    int AutoRun() override;

private:

    bool AutoDone = false;

    units::meters_per_second_t ratioSpeedCalc(units::length::meter_t dx, units::length::meter_t dt, units::meters_per_second_t speed)
    {
        if(dx.value() == 0 || dt.value() == 0 || speed.value() == 0)
        {
            return 0_mps;
        }
        return (dx / dt) * speed;
    }

    units::meters_per_second_squared_t ratioAccelCalc(units::length::meter_t dx, units::length::meter_t dt, units::meters_per_second_squared_t accel)
    {
        if(dx.value() == 0 || dt.value() == 0 || accel.value() == 0)
        {
            return 0_mps_sq;
        }
        return (dx / dt) * accel;
    }

    Drivetrain &Swervedrive;

    // [TODO] these can all be smart pointers
    frc::TrapezoidProfile<units::meters>::Constraints fwdConstraints{0_mps, 0_mps_sq};
    frc::TrapezoidProfile<units::meters>::State fwdStartState{0_m, 0_mps};
    frc::TrapezoidProfile<units::meters>::State fwdEndState{0_m, 0_mps};
    frc::TrapezoidProfile<units::meters> fwdProfile{fwdConstraints, fwdEndState, fwdStartState};
    
    frc::TrapezoidProfile<units::meters>::Constraints strConstraints{0_mps, 0_mps_sq};
    frc::TrapezoidProfile<units::meters>::State strStartState{0_m, 0_mps};
    frc::TrapezoidProfile<units::meters>::State strEndState{0_m, 0_mps};
    frc::TrapezoidProfile<units::meters> strProfile{strConstraints, strEndState, strStartState};

    frc::TrapezoidProfile<units::degree>::Constraints RCWConstraints{0_deg_per_s, 0_deg_per_s_sq};
    frc::TrapezoidProfile<units::degree>::State RCWStartState{0_deg, 0_deg_per_s};
    frc::TrapezoidProfile<units::degree>::State RCWEndState{0_deg, 0_deg_per_s};
    frc::TrapezoidProfile<units::degree> RCWProfile{RCWConstraints, RCWEndState, RCWStartState};


    units::meter_t fwd;
    units::meter_t right;
    units::radian_t Theta;

    std::function<void(void)> customAutoCB; // Use function type

    frc::SlewRateLimiter<units::radians> rcwSlew{Theta / fwdProfile.TotalTime()}; // rotates the robot over the course of the distance it will be traveling in that step
};

#endif