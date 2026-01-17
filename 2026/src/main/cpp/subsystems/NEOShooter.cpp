#include "subsystems/NEOShooter.h"

#define kLeftNEOMotorID 58
#define kRightNEOMotorID 59

using namespace ShooterConstants;
using namespace CANConstants;

NEOShooter::NEOShooter()
{

  LeftMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kLeftNEOMotorID);
  RightMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kRightNEOMotorID);

  SetName("NEOShooter");
  
  SetDefaultCommand(frc2::cmd::Run([this] {SetShooterSpeeds(0);}, {this}));
}

void NEOShooter::SetShooterSpeeds(double voltage) 
{
    LeftMotor->Set(-voltage);
    RightMotor->Set(voltage);
}


frc2::CommandPtr NEOShooter::SetShootSpeed()
{
  return frc2::FunctionalCommand(
    [this] {},
    [this] {SetShooterSpeeds(1);},
    [this] (bool interrupted){SetShooterSpeeds(0);},
    [this] {SetShooterSpeeds(0); return (false);},
    {this}
  ).ToPtr();
}


