#include "subsystems/Shooter.h"
#include "Constants.h"
#include "rev/CANSparkMax.h"
#include <frc2/command/button/NetworkButton.h>


using namespace CANConstants;

Shooter::Shooter()
{

  TopShooterMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kShooterLowerMoterID);
  TopShooterMotor->SetInverted(true);
  pid.kV = ShooterConstants::kTopShooterFF;
  pid.kP = ShooterConstants::kTopShooterP;
  pid.kI = ShooterConstants::kTopShooterI;
  pid.kD = ShooterConstants::kTopShooterD;
  TopShooterMotor->GetConfigurator().Apply(pid);
  
  BottomShooterMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kShooterUpperMotorID);
  BottomShooterMotor->SetInverted(true);
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
  #if BOT == CompBot
  BottomShooterMotor->SetControl(VelocityClosedLoop.WithVelocity(RPM));
  #else
  BottomShooterPID->SetReference(RPM.value(), rev::CANSparkMax::ControlType::kVelocity);
  #endif
}

void Shooter::SetTopMotorRPM(units::turns_per_second_t RPM)
{
  #if BOT == CompBot
  TopShooterMotor->SetControl(VelocityClosedLoop.WithVelocity(RPM));
  #else
  TopShooterPID->SetReference(RPM.value(), rev::CANSparkMax::ControlType::kVelocity);
  #endif
}

units::turns_per_second_t Shooter::GetBottomMotorRPM()
{
  #if BOT == CompBot
  return BottomShooterMotor->GetVelocity().GetValue();
  #else
  return units::turns_per_second_t{BottomShooterEncoder->GetVelocity()};
  #endif
}

units::turns_per_second_t Shooter::GetTopMotorRPM()
{
  #if BOT == CompBot
  return TopShooterMotor->GetVelocity().GetValue();
  #else
  return units::turns_per_second_t{TopShooterEncoder->GetVelocity()};
  #endif

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


