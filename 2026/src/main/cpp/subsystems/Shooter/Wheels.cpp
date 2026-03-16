#include "subsystems/Shooter/Wheels.h"

ShooterWheels::ShooterWheels()
{
  LeftMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kLeftShooterID);
  RightMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kRightShooterID);
  LeftMotor->SetNeutralMode(0);
  RightMotor->SetNeutralMode(0);

  pid.kV = ShooterConstants::kShooterFF;
  pid.kP = ShooterConstants::kShooterP;
  pid.kI = ShooterConstants::kShooterI;
  pid.kD = ShooterConstants::kShooterD;
  LeftMotor->GetConfigurator().Apply(pid);
  RightMotor->GetConfigurator().Apply(pid);

  SetName("ShooterWheels");
  
  //SetDefaultCommand(frc2::cmd::Run([this] {setSpeed(0_tps);}, {this}));
}

void ShooterWheels::Periodic()
{
  // auto distance = Turret_Sys->GetDistanceMeters();
  ShootSpeed_ = units::turns_per_second_t(5.31 * 5 + 37.95);
  // ShootSpeed_ = units::turns_per_second_t(5.31 * distance + 37.95);
}

void ShooterWheels::setSpeed(units::turns_per_second_t TPS) 
{
    ctre::phoenix6::controls::VelocityVoltage m_velocity{0_tps};
    LeftMotor->SetControl(m_velocity.WithVelocity(-TPS));
    RightMotor->SetControl(m_velocity.WithVelocity(TPS));
}

void ShooterWheels::setNeutral()
{
  LeftMotor->SetVoltage(0_V);
  RightMotor->SetVoltage(0_V);
}

frc2::CommandPtr ShooterWheels::Spin()
{
    return frc2::FunctionalCommand(
    [this] {},
    [this] {
      setSpeed(ShootSpeed_);},
    [this] (bool interrupted) {},
    [this] {return (RightMotor->GetVelocity().GetValue().convert<units::turns_per_second>() >= ShootSpeed_.convert<units::turns_per_second>());},
    {this}
  ).ToPtr();
}

frc2::CommandPtr ShooterWheels::shootToDistance(std::function<void()> getDistance)
{
    return frc2::FunctionalCommand(
    [this] {},
    [this] {
      setSpeed(ShootSpeed_);},
    [this] (bool interrupted) {},
    [this] {return (RightMotor->GetVelocity().GetValue().convert<units::turns_per_second>() >= ShootSpeed_.convert<units::turns_per_second>());},
    {this}
  ).ToPtr();
}

frc2::CommandPtr ShooterWheels::Stop()
{
    return frc2::FunctionalCommand(
    [this] {},
    [this] {setNeutral();},
    [this] (bool interrupted) {},
    [this] {return true;},
    {this}
  ).ToPtr();
}

