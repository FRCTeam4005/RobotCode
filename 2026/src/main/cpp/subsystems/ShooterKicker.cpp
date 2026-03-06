#include "subsystems/ShooterKicker.h"


using namespace ShooterConstants;
using namespace CANConstants;

ShooterKicker::ShooterKicker()
{
  KickerMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kKickerMotorID);
  KickerMotor->SetNeutralMode(0);

  SetName("ShooterKicker");

  SetDefaultCommand(frc2::cmd::Run([this] {KickerMotor->Set(0);}, {this}));
}

frc2::CommandPtr ShooterKicker::Feed()
{
  return frc2::cmd::RunOnce([this](){KickerMotor->Set(-1);});
}

frc2::CommandPtr ShooterKicker::Stop()
{
  return frc2::cmd::RunOnce([this](){KickerMotor->Set(0);});
}
