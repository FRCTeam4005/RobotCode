#include "subsystems/Elevator.h"
#include "frc2/command/FunctionalCommand.h"
#include "frc/smartdashboard//SmartDashboard.h"
#include <iostream>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <cmath>

using namespace ElevatorConstants;

Elevator::Elevator(CANDigitalInput *elevatormagswitch) 
: ElevatorMagSwitch(elevatormagswitch)
{
    ElevateMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kElevatorMotorID);
    ctre::phoenix6::configs::TalonFXConfiguration elevate_cfg{};
    ctre::phoenix6::configs::MotionMagicConfigs &mm = elevate_cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 3000_tps; // 5 (mechanism) rotations per second cruise
    mm.MotionMagicAcceleration = 200_tr_per_s_sq; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.1 seconds to reach max accel 
    mm.MotionMagicJerk = 200_tr_per_s_cu;

    elevate_cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    elevate_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevate_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = units::angle::turn_t(10);
    elevate_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    elevate_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = units::angle::turn_t(-135);

    elevate_cfg.CurrentLimits.StatorCurrentLimit = units::current::ampere_t(80);
    elevate_cfg.CurrentLimits.StatorCurrentLimitEnable = false;

    ctre::phoenix6::configs::Slot0Configs &slot0_ = elevate_cfg.Slot0;
    slot0_.kS = 0; // Add 0.25 V output to overcome static friction
    slot0_.kV = 15; // A velocity target of 1 rps results in 0.12 V output
    slot0_.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0_.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0_.kI = 0; // No output for integrated error
    slot0_.kD = 0; // A velocity error of 1 rps results in 0.5 V output

  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  status = ElevateMotor->GetConfigurator().Apply(elevate_cfg);

  CoralSensor = std::make_unique<frc::DigitalInput>(0);
  RangeSensor = std::make_unique<ctre::phoenix6::hardware::CANrange>(RangeSensorPort);
  range_config = ctre::phoenix6::configs::CANrangeConfiguration();
  range_config.ToFParams.UpdateMode.LongRangeUserFreq;
  range_config.ProximityParams.WithProximityThreshold(4_m);
  RangeSensor->GetConfigurator().Apply(range_config);

  ElevateMotor->SetPosition(units::turn_t(0));
  currentLevel = MAX_LEVELS;
  SetName("Elevator");
}

frc2::CommandPtr Elevator::SetToLevel(Level level)
{
    return frc2::FunctionalCommand(
        [level, this]{
            switch (level) {
                case L2: 
                    SetToL2();
                break;

                case L3: 
                    SetToL3();
                break;

                case L4:
                    SetToL4();
                break;

                case L2_Algae:
                    SetToL2_Algae();
                break;

                case L3_Algae:
                    SetToL3_Algae();
                break;

                case READY:
                    SetToReady();
                break;

                default: 

                break;
            }
        },
        [this]{},
        [this](bool interrupted){},
        [this]{return (std::fabs(goalTurns.value() - GetPosition().value()) < 1);},
        {this}
    ).ToPtr();
}

frc2::CommandPtr Elevator::BumpUp() {
    return frc2::FunctionalCommand(
    [this] {},
    [this] {ElevateMotor->Set(-0.3);},
    [this] (bool interrupted) {
        ElevateMotor->Set(0);},
    [this] {
        return true;},
    {}
  ).ToPtr();
}

frc2::CommandPtr Elevator::BumpDown() {
    return frc2::FunctionalCommand(
    [this] {},
    [this] {ElevateMotor->Set(0.3);},
    [this] (bool interrupted) {
        ElevateMotor->Set(0);},
    [this] {
        return true;},
    {}
  ).ToPtr();
}

bool Elevator::CanScore()
{
    return (std::fabs(RangeSensor->GetDistance().GetValueAsDouble() - kCanScore) < 0.0325);
}

bool Elevator::IsCoralInIntake()
{
  return !CoralSensor->Get();
}

void Elevator::SetElevatorCommand(units::turn_t goal) {
    ElevateMotor->SetControl(elevate_mmReq.WithPosition(goal).WithSlot(0));
}

Level Elevator::GetState() 
{
    return currentLevel;
}

units::turn_t Elevator::GetPosition() {
    return ElevateMotor->GetRotorPosition().GetValue();
}


frc2::CommandPtr Elevator::SetToCollect() {
    currentLevel = L2;
    return frc2::FunctionalCommand(
    [this] {},
    [this] {ElevateMotor->Set(0.9);},
    [this] (bool interrupted) {
        ElevateMotor->SetPosition(units::turn_t(-15));
        ElevateMotor->Set(0);},
    [this] {
        return (ElevatorMagSwitch->IsElevatorMagSensorActive());},
    {}
  ).ToPtr();
}

void Elevator::SetToL2() {
    goalTurns = ElevatorConstants::kL2;
    currentLevel = L2;
    SetElevatorCommand(goalTurns);
}

void Elevator::SetToL3() {
    goalTurns = ElevatorConstants::kL3;
    currentLevel = L3;
    SetElevatorCommand(goalTurns);
}

void Elevator::SetToL4() {
    goalTurns = ElevatorConstants::kL4;
    currentLevel = L4;
    SetElevatorCommand(goalTurns);
}

void Elevator::SetToL2_Algae() {
    goalTurns = ElevatorConstants::kL2_Algae;
    currentLevel = L2_Algae;
    SetElevatorCommand(goalTurns);
}

void Elevator::SetToL3_Algae() {
    goalTurns = ElevatorConstants::kL3_Algae;
    currentLevel = L3_Algae;
    SetElevatorCommand(goalTurns);
}

void Elevator::SetToReady() {
    goalTurns = ElevatorConstants::kReady;
    currentLevel = READY;
    SetElevatorCommand(goalTurns);
}
