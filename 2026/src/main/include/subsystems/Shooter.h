
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/voltage.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <ctre/phoenix6/TalonFX.hpp>

class Shooter : public frc2::SubsystemBase
{
 public:
  Shooter();

  auto SetShootSpeed(units::turns_per_second_t topWheelTPS, units::turns_per_second_t bottomWheelTPS) -> frc2::CommandPtr;
  void SetBottomMotorRPM(units::turns_per_second_t RPM);
  void SetTopMotorRPM(units::turns_per_second_t RPM);
  
private:
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> BottomShooterMotor;
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> TopShooterMotor;

  ctre::phoenix6::configs::Slot0Configs pid;
  ctre::phoenix6::controls::VelocityVoltage VelocityClosedLoop{0_tps};

  units::turns_per_second_t GetBottomMotorRPM();
  units::turns_per_second_t GetTopMotorRPM();

  void SetShooterSpeeds(units::turns_per_second_t TopRPM, units::turns_per_second_t BottomRPM);

  void ShooterInitialize();
  
  bool IsShooterDone(units::turns_per_second_t DesiredTopRPM, units::turns_per_second_t DesiredBottomRPM);


  
};