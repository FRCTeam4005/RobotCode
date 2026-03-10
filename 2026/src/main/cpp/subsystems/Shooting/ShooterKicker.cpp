#include "subsystems/Shooting/ShooterKicker.h"


using namespace ShooterConstants;
using namespace CANConstants;

ShooterKicker::ShooterKicker()
{
  KickerMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kKickerMotorID);
  KickerMotor->SetNeutralMode(0);

  SetName("ShooterKicker");

  //SetDefaultCommand(frc2::cmd::Run([this] {KickerMotor->Set(0);}, {this}));
}

frc2::CommandPtr ShooterKicker::Feed()
{
    return frc2::FunctionalCommand(
    [this] {},
    [this] {
      KickerMotor->Set(-1);},
    [this] (bool interrupted) {},
    [this] {return true;},
    {this}
  ).ToPtr();
}

frc2::CommandPtr ShooterKicker::Stop()
{
    return frc2::FunctionalCommand(
    [this] {},
    [this] {KickerMotor->Set(0);},
    [this] (bool interrupted) {},
    [this] {return true;},
    {this}
  ).ToPtr();
}