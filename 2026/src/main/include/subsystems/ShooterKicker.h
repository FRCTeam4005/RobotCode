// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include "generated/TunerConstants.h"
#include <ctre/phoenix6/TalonFX.hpp>

class ShooterKicker : public frc2::SubsystemBase 
{
 public:
  ShooterKicker();
 
  auto Feed() -> frc2::CommandPtr;
  auto Stop() -> frc2::CommandPtr;
  auto On() -> frc2::CommandPtr;
  auto Off() -> frc2::CommandPtr;
  auto Jog() -> frc2::CommandPtr;
  
private:
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> KickerMotor;

  auto JogIn() -> void;
  auto JogOut() -> void;
  void setSpeed(double speed);
  void SetKicker(  double voltage);
};
