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

  pid.kV = ShooterConstants::kBottomShooterFF;
  pid.kP = ShooterConstants::kBottomShooterP;
  pid.kI = ShooterConstants::kBottomShooterI;
  pid.kD = ShooterConstants::kBottomShooterD;
  LeftMotor->GetConfigurator().Apply(pid);
  RightMotor->GetConfigurator().Apply(pid);

  SetName("Shooter");
  
  SetDefaultCommand(frc2::cmd::Run([this] {SetShooterSpeeds(0_tps);}, {this}));
}

void Shooter::SetShooterSpeeds(units::turns_per_second_t TPS) 
{
    ctre::phoenix6::controls::VelocityVoltage m_velocity{0_tps};
    LeftMotor->SetControl(m_velocity.WithVelocity(TPS));
    RightMotor->SetControl(m_velocity.WithVelocity(-TPS));
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
      SetShooterSpeeds(70_tps);},
    [this] (bool interrupted){
      SetKicker(0.0);
      SetShooterSpeeds(0_tps);},
    [this] {return (false);},
    {this}
  ).ToPtr();
}


