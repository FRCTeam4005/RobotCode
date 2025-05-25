#pragma once

#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/Commands.h>
#include <frc2/command/TrapezoidProfileSubsystem.h>
#include <units/angle.h>
#include <Constants.h>
#include <RobotConstants.h>
#include <frc/DigitalInput.h>
#include <units/Angle.h>
#include <units/angular_velocity.h>
#include <units/Time.h>
#include <frc2/command/CommandPtr.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "subsystems/CANdi.h"
#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/SmartDashboard.h>

enum class ClawPosition { COLLECT, SCORE, PATH };

class Claw : public frc2::SubsystemBase {
public:
    Claw(CANDigitalInput *clawBreakBeam);
    auto SetPosition(ClawPosition position) -> frc2::CommandPtr;
    auto SetToCollect() -> frc2::CommandPtr;

private:
    void Periodic () override 
    {
        frc::SmartDashboard::PutNumber("ClawPosition:", GetPosition().value());
        frc::SmartDashboard::PutBoolean("ClawBreakBeam", ClawBreakBeam->IsClawBreakBeamActive());
    }
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> ClawMotor;
    ctre::phoenix6::controls::MotionMagicVoltage m_mmReq{0_tr};
    ctre::phoenix6::configs::MotorOutputConfigs clawMotorCFG;

    CANDigitalInput *ClawBreakBeam;
    
    auto GetPosition() -> units::turn_t;
    auto SetToScore() -> void;
    auto SetToPath() -> void;
};