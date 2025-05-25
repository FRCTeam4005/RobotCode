#pragma once

#include <frc/controller/ArmFeedforward.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>
#include <frc2/command/TrapezoidProfileSubsystem.h>
#include <units/angle.h>
#include <Constants.h>
#include <RobotConstants.h>
#include "rev/SparkMax.h"
#include <frc/DigitalInput.h>
#include <units/Angle.h>
#include <units/angular_velocity.h>
#include <units/Time.h>
#include <frc2/command/CommandPtr.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/button/Trigger.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <ctre/phoenix6/CANrange.hpp>
#include "CANdi.h"

enum Level {L2, L3, L4, READY, L2_Algae, L3_Algae, MAX_LEVELS};

class Elevator : public frc2::SubsystemBase {
public:
    Elevator(CANDigitalInput *elevatormagswitch);
    auto SetToLevel(Level level) -> frc2::CommandPtr;
    auto SetToCollect() -> frc2::CommandPtr;
    auto Up() -> frc2::CommandPtr;
    auto Down() -> frc2::CommandPtr;
    auto Score() -> frc2::CommandPtr;
    auto GetState() -> Level;
    auto GetPosition() -> units::angle::turn_t;
    auto IsCoralInIntake() -> bool;
    auto BumpUp() -> frc2::CommandPtr;
    auto BumpDown() -> frc2::CommandPtr;

    frc2::Trigger CoralReady()
	{
		return frc2::Trigger([this] {
			return (IsCoralInIntake() && (GetState() == Level::READY) && (frc::DriverStation::IsTeleopEnabled()));
		});
	}

    frc2::Trigger L4Score()
    {
        return frc2::Trigger([this] {
            return (CanScore() && (GetState() == Level::L4) && (RangeSensor->GetIsDetected().GetValue()) && (frc::DriverStation::IsTeleopEnabled()));
        });
    }

private:
    void Periodic () override 
    {
        frc::SmartDashboard::PutNumber("elevatorPosition:", GetPosition().value());
        frc::SmartDashboard::PutBoolean("ElevatorMagSwitch", ElevatorMagSwitch->IsElevatorMagSensorActive());
        frc::SmartDashboard::PutBoolean("CoralInSensor", IsCoralInIntake());
        frc::SmartDashboard::PutNumber("Range Sensor:", RangeSensor->GetDistance().GetValue().value());
    }

    Level currentLevel = MAX_LEVELS;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> ElevateMotor;
    ctre::phoenix6::controls::MotionMagicVoltage elevate_mmReq{0_tr};

    CANDigitalInput *ElevatorMagSwitch;
    std::unique_ptr<frc::DigitalInput> CoralSensor;
    std::unique_ptr<ctre::phoenix6::hardware::CANrange> RangeSensor;
    ctre::phoenix6::configs::CANrangeConfiguration range_config;

    units::turn_t goalTurns;
    void SetElevatorCommand(units::turn_t goal);
    auto SetToL2() -> void;
    auto SetToL3() -> void;
    auto SetToL4() -> void;
    auto SetToL2_Algae() -> void;
    auto SetToL3_Algae() -> void;
    auto SetToReady() -> void;
    
    auto CanScore() -> bool;
};