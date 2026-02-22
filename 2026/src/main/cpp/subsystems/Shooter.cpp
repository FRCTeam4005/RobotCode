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

  LeftMotor->SetNeutralMode(0);
  RightMotor->SetNeutralMode(0);

  pid.kV = ShooterConstants::kShooterFF;
  pid.kP = ShooterConstants::kShooterP;
  pid.kI = ShooterConstants::kShooterI;
  pid.kD = ShooterConstants::kShooterD;
  LeftMotor->GetConfigurator().Apply(pid);
  RightMotor->GetConfigurator().Apply(pid);

  SetName("Shooter");
  
  SetDefaultCommand(frc2::cmd::Run([this] {SetShooterSpeeds(0_tps);}, {this}));
}

void Shooter::SetShooterSpeeds(units::turns_per_second_t TPS) 
{

  if(TPS.value() != 0)
  {
    ctre::phoenix6::controls::VelocityVoltage m_velocity{0_tps};
    LeftMotor->SetControl(m_velocity.WithVelocity(-TPS));
    RightMotor->SetControl(m_velocity.WithVelocity(TPS));
  }
  else 
  {
    LeftMotor->SetVoltage(0_V);
    RightMotor->SetVoltage(0_V);
  }

}

void Shooter::SetKicker(double voltage)
{
  KickerMotor->Set(voltage);
}

frc2::CommandPtr Shooter::SetShootSpeed(units::turns_per_second_t speed)
{
  return frc2::FunctionalCommand(
    [this] {},
    [speed, this] {
      SetShooterSpeeds(speed);},
    [this] (bool interrupted) {},
    [speed, this] {return (GetShooterSpeed() > double(speed));},
    {this}
  ).ToPtr();
}

frc2::CommandPtr Shooter::FeedShooter()
{
  return frc2::FunctionalCommand(
    [this] {},
    [this] {
      frc::SmartDashboard::SmartDashboard::PutNumber("asdasdfsadfsa",-1.0);
      SetKicker(-1.0);
    },
    [this] (bool interrupted){
      SetKicker(0.0);
      },
    [this] {return (false);},
    {this}
  ).ToPtr();
}

double Shooter::GetShooterSpeed() 
{
  return RightMotor->GetVelocity().GetValueAsDouble();
}



