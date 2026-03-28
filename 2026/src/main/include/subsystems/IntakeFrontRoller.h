
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "generated/TunerConstants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DoubleSolenoid.h>


class IntakeFrontRoller : public frc2::SubsystemBase
{
 public:
  IntakeFrontRoller();


  auto Unstick() -> frc2::CommandPtr; //Pick up fuel, intake in
  auto Out() -> frc2::CommandPtr; //Feed fuel to shooter
  auto StopIntake() -> frc2::CommandPtr; //Feed fuel to shooter
  auto Momentary() -> frc2::CommandPtr; // its momentaary
  auto Stop() -> frc2::CommandPtr;



private:
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> IntakeFrontRollerMotor;
  frc::DoubleSolenoid m_doubleSolenoid;
  void setSpeed(double speed);


  auto IntakeIn() -> void;
  auto IntakeOut() -> void;
  auto RollerJog(double) -> void;
  auto RollerIn() -> void;
  auto RollerOut() -> void;
};