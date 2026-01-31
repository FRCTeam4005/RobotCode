
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "generated/TunerConstants.h"
#include <units/angle.h>
#include <frc2/command/Commands.h>
#include <units/voltage.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

class Shooter : public frc2::SubsystemBase 
{
 public:
  Shooter();

  auto SetShootSpeed() -> frc2::CommandPtr;
  
  
private:

  void Periodic() override
  {
    frc::SmartDashboard::PutNumber("Shooter Velocity (Turns per Second)", LeftMotor->GetVelocity().GetValueAsDouble());
  }

  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> LeftMotor;
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> RightMotor;
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> KickerMotor;

  units::turns_per_second_t GetShooterSpeed();
  
  void SetShooterSpeeds(units::turns_per_second_t speed) ;

  void SetShooterSpeeds(double speed) ;

  void SetKicker(double voltage);
};