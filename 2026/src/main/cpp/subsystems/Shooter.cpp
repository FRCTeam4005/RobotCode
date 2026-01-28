#include "subsystems/Shooter.h"

// #define kLeftNEOMotorID 59
// #define kRightNEOMotorID 58

using namespace ShooterConstants;
using namespace CANConstants;

Shooter::Shooter()
{

  LeftMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kLeftShooterID);
  RightMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kRightShooterID);
  KickerMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kKickerMotorID);

  SetName("Shooter");
  
  SetDefaultCommand(frc2::cmd::Run([this] {SetShooterSpeeds(0);}, {this}));
}

void Shooter::SetShooterSpeeds(double voltage) 
{
    LeftMotor->Set(voltage);
    RightMotor->Set(-voltage);
}

void Shooter::SetKicker(double voltage)
{
  KickerMotor->Set(voltage);
}

frc2::CommandPtr Shooter::SetShootSpeed()
{
  return frc2::FunctionalCommand(
    [this] {},
    [this] {
      SetKicker(-1.0);
      SetShooterSpeeds(.58);},
    [this] (bool interrupted){
      SetKicker(0.0);
      SetShooterSpeeds(0);},
    [this] {return (false);},
    {this}
  ).ToPtr();
}


