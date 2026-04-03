
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "generated/TunerConstants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

class IntakeConveyor : public frc2::SubsystemBase
{
 public:
  IntakeConveyor ();
  auto In() -> frc2::CommandPtr; //Pick up fuel, intake in
  auto Out() -> frc2::CommandPtr; //Feed fuel to shooter
  auto Stop() -> frc2::CommandPtr; //Stop intaking

private:
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> IntakeConveyorMotor;
  std::unique_ptr<ctre::phoenix::motorcontrol::can::TalonSRX> ConveyorBiasWheel;
  //std::unique_ptr<ctre::phoenix6::hardware::TalonFX> ConveyorBiasWheel;
  void setSpeed(double speed);
};
