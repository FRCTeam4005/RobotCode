#include "subsystems/Shooter.h"


using namespace ShooterConstants;
using namespace CANConstants;

Shooter::Shooter(Turret* turret_sys)
:m_doubleSolenoid( CANConstants::kPneumaticHub,
                   frc::PneumaticsModuleType::REVPH,
                   PneumaticsChannelConst::kShooterDownChannel,
                   PneumaticsChannelConst::kShooterUpChannel )
{
  Turret_Sys = turret_sys;
  LeftMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kLeftShooterID);
  RightMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kRightShooterID);
  KickerMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kKickerMotorID);
  m_doubleSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
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
      autoSpeed = units::turns_per_second_t(5.31 * distance + 37.95);
      SetShooterSpeeds(autoSpeed);},
    [this] (bool interrupted) {},
    [this] {return (GetShooterSpeed() > double(autoSpeed));},
    {this}
  ).ToPtr();
}

frc2::CommandPtr Shooter::FeedShooter()
{
  return frc2::FunctionalCommand(
    [this] {},
    [this] {
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

frc2::CommandPtr Shooter::ShooterToggle()
{
  return this->RunOnce(
    [this] {m_doubleSolenoid.Toggle(); }
  );

}
frc2::CommandPtr Shooter::ShooterUp()
{
  return this->RunOnce(
    [this] {m_doubleSolenoid.Set(frc::DoubleSolenoid::Value::kForward); }
  );

}
frc2::CommandPtr Shooter::ShooterDown()
{
  return this->RunOnce(
    [this] {m_doubleSolenoid.Set(frc::DoubleSolenoid::Value::kReverse); }
  );

}

void Shooter::ShooterUpCommand()
{
    m_doubleSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}
void Shooter::ShooterDownCommand()
{
  m_doubleSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}


