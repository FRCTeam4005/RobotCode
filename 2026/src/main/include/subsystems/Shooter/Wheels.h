// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "generated/TunerConstants.h"
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "subsystems/Shooter/Wheels.h"
#include <functional>

class ShooterWheels : public frc2::SubsystemBase 
{
 public:
  ShooterWheels();
  auto Toggle() -> frc2::CommandPtr;
  auto Spin() -> frc2::CommandPtr;
  auto Stop() -> frc2::CommandPtr;
  auto shootToDistance(std::function<void()> getDistance) -> frc2::CommandPtr;
  
private:
  units::turns_per_second_t ShootSpeed_;

  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> LeftMotor;
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> RightMotor;
  ctre::phoenix6::configs::Slot0Configs pid;

  void Periodic() override;
  void setSpeed(units::turns_per_second_t speed);
  void setNeutral();
};
