
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"
#include <units/angle.h>
#include <frc2/command/Commands.h>
#include <frc2/command/TrapezoidProfileSubsystem.h>
#include "rev/CANSparkMax.h"
#include <units/voltage.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <frc/shuffleboard/Shuffleboard.h>

class Shooter : public frc2::SubsystemBase 
{
 public:
  Shooter();

  auto SetShootSpeed(units::turns_per_second_t topWheelTPS, units::turns_per_second_t bottomWheelTPS) -> frc2::CommandPtr;
  void SetBottomMotorRPM(units::turns_per_second_t RPM);
  void SetTopMotorRPM(units::turns_per_second_t RPM);
  
private:
  #if BOT == CompBot
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> BottomShooterMotor;
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> TopShooterMotor;
  #else
  std::unique_ptr<rev::CANSparkMax> BottomShooterMotor;
  std::unique_ptr<rev::CANSparkMax> TopShooterMotor;

  std::unique_ptr<rev::SparkRelativeEncoder> BottomShooterEncoder;
  std::unique_ptr<rev::SparkRelativeEncoder> TopShooterEncoder;

  std::unique_ptr<rev::SparkPIDController> BottomShooterPID;
  std::unique_ptr<rev::SparkPIDController> TopShooterPID;

  #endif
  ctre::phoenix6::configs::Slot0Configs pid;
  ctre::phoenix6::controls::VelocityVoltage VelocityClosedLoop{0_tps, 0_tr_per_s_sq, true, 0_V, 0, false};

  units::turns_per_second_t GetBottomMotorRPM();
  units::turns_per_second_t GetTopMotorRPM();

  void SetShooterSpeeds(units::turns_per_second_t TopRPM, units::turns_per_second_t BottomRPM);

  void ShooterInitialize();
  
  bool IsShooterDone(units::turns_per_second_t DesiredTopRPM, units::turns_per_second_t DesiredBottomRPM);


  
};