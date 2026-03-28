#include "subsystems/Shooter/Wheels.h"

ShooterWheels::ShooterWheels(std::function<frc::Pose2d()> getBotPose):
_getPose{getBotPose}
{
  LeftMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kLeftShooterID);
  RightMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kRightShooterID);
  
  
  ctre::phoenix6::configs::TalonFXConfiguration LeftMotorcfg{};
  ctre::phoenix6::configs::TalonFXConfiguration RightMotorcfg{};

  LeftMotor->SetNeutralMode(0);
  RightMotor->SetNeutralMode(0);

  setupControllerGains(LeftMotorcfg);
  setupControllerGains(RightMotorcfg);
  
  RightMotorcfg.MotorOutput.Inverted = false;
  LeftMotorcfg.MotorOutput.Inverted = true;

  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  status = LeftMotor->GetConfigurator().Apply(LeftMotorcfg);
  status = RightMotor->GetConfigurator().Apply(RightMotorcfg);
    
  SetName("ShooterWheels");
  
  //SetDefaultCommand(frc2::cmd::Run([this] {setSpeed(0_tps);}, {this}));
}

void ShooterWheels::Periodic()
{

  auto CurrPose = _getPose();
  auto TargetLocation = getTargetTranlation(CurrPose);
  auto TargetCoords = getTargetDistance(TargetLocation,CurrPose);

  ShootSpeed_ = units::turns_per_second_t(5.6 * TargetCoords.value() + 37.95);
  // ShootSpeed_ = units::turns_per_second_t(5.31 * distance + 37.95);
}

void ShooterWheels::setSpeed(units::turns_per_second_t TPS) 
{
    ctre::phoenix6::controls::VelocityVoltage m_velocity{0_tps};
    LeftMotor->SetControl(m_velocity.WithVelocity(TPS));
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

