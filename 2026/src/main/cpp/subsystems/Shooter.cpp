#pragma once

#include "subsystems/Shooter.h"
#include "generated/TunerConstants.h"
#include <frc2/command/button/NetworkButton.h>


using namespace CANConstants;

Shooter::Shooter()
{

  TopShooterMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kShooterLowerMoterID);
  pid.kV = ShooterConstants::kTopShooterFF;
  pid.kP = ShooterConstants::kTopShooterP;
  pid.kI = ShooterConstants::kTopShooterI;
  pid.kD = ShooterConstants::kTopShooterD;
  TopShooterMotor->GetConfigurator().Apply(pid);
  
  BottomShooterMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kShooterUpperMotorID);
  pid.kV = ShooterConstants::kBottomShooterFF;
  pid.kP = ShooterConstants::kBottomShooterP;
  pid.kI = ShooterConstants::kBottomShooterI;
  pid.kD = ShooterConstants::kBottomShooterD;
  BottomShooterMotor->GetConfigurator().Apply(pid);
  SetName("Shooter");
  
  SetDefaultCommand(frc2::cmd::Run([this] {SetShooterSpeeds(0_tps, 0_tps);}, {this}));
}

void Shooter::SetShooterSpeeds(units::turns_per_second_t TopRPM, units::turns_per_second_t BottomRPM)
{
  SetTopMotorRPM(TopRPM);
  SetBottomMotorRPM(BottomRPM); 
}

void Shooter::SetBottomMotorRPM(units::turns_per_second_t RPM)
{
  BottomShooterMotor->SetControl(VelocityClosedLoop.WithVelocity(RPM));

}

void Shooter::SetTopMotorRPM(units::turns_per_second_t RPM)
{
  TopShooterMotor->SetControl(VelocityClosedLoop.WithVelocity(RPM));
}

units::turns_per_second_t Shooter::GetBottomMotorRPM()
{
  return BottomShooterMotor->GetVelocity().GetValue();
}

units::turns_per_second_t Shooter::GetTopMotorRPM()
{
  return TopShooterMotor->GetVelocity().GetValue();
}

bool Shooter::IsShooterDone(units::turns_per_second_t DesiredTopRPM, units::turns_per_second_t DesiredBottomRPM)
{
  return (DesiredBottomRPM.value() >= GetBottomMotorRPM().value()) && (DesiredTopRPM.value() >= GetTopMotorRPM().value());
}

frc2::CommandPtr Shooter::SetShootSpeed(units::turns_per_second_t topWheelTPS, units::turns_per_second_t bottomWheelTPS)
{
  return frc2::FunctionalCommand(
    [this] {},
    [topWheelTPS, bottomWheelTPS, this] {SetShooterSpeeds(topWheelTPS, bottomWheelTPS);},
    [this] (bool interrupted){},
    [topWheelTPS, this] {return (GetTopMotorRPM() > topWheelTPS);},
    {this}
  ).ToPtr();
}


