// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/Commands.h>
#include <frc2/command/TrapezoidProfileSubsystem.h>
#include <units/angle.h>
#include <Constants.h>
#include "rev/CANSparkMax.h"
#include <frc/DigitalInput.h>
#include <units/Angle.h>
#include <units/angular_velocity.h>
#include <units/Time.h>
#include <frc2/command/CommandPtr.h>
#include <frc/shuffleboard/Shuffleboard.h>

class Arm : public frc2::TrapezoidProfileSubsystem<units::turn> {

  using State = frc::TrapezoidProfile<units::turn>::State;


 public:

  Arm();
  void UseState(State setpoint) override;
  frc2::CommandPtr SetArmGoalCommand(units::turn_t goal);
  auto SetToClimbPosition() -> frc2::CommandPtr;
  auto SetAboveBumperPosition() -> frc2::CommandPtr;
  auto SetOnSwervePosition() -> frc2::CommandPtr;
  auto SetBelowBumperPosition() -> frc2::CommandPtr;
  auto SetDesiredPosition(units::turn_t desiredPosition) -> frc2::CommandPtr;
  auto SetSlightlyAboveFloor() -> frc2::CommandPtr;
  auto GetPosition() -> units::turn_t;


 private:

  std::unique_ptr<rev::CANSparkMax> m_RotLeft;
  std::unique_ptr<rev::CANSparkMax> m_RotRight;

  std::unique_ptr<rev::SparkPIDController> RotLeftPID;
  std::unique_ptr<rev::SparkPIDController> RotRightPID;

  std::unique_ptr<rev::SparkRelativeEncoder> RotLeftEncoder;
  std::unique_ptr<rev::SparkRelativeEncoder> RotRightEncoder;

  //Assume we're starting on top of swerve to begin match

  const std::string S_ArmDebug = "Arm Position";
  
  void InitSendable(wpi::SendableBuilder& builder) override;
  void SetPosition(units::turn_t postion);
  void SetSlewedReferencePosition(units::turn_t position);
  void SetReferencePosition(units::turn_t position);
};