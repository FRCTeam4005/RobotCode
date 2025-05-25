#include "subsystems/Climber.h"
#include "frc2/command/FunctionalCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/time.h>
#include <frc/Timer.h>

using namespace ctre::phoenix6;
using namespace units::angle;

Climber::Climber(Pneumatics *Pneumatics_Sys)
{
  climberMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kClimberLeftID);
  
  Climber_Pawl_Sol = std::make_unique<frc::DoubleSolenoid>(Pneumatics_Sys->GetDoubleSolenoid(2,3));


  ctre::phoenix6::configs::Slot0Configs  climberMotorConfig{};

  //PID to shoot as fast a possible 
  //On climb allow driver to
  //Once going up, lock it.

  //Get in a postion and send in a positon to see how well it did
  //Get a fast enough 

  climberMotorConfig.kP = 0.05; //proportional
  climberMotorConfig.kI = 0.00002; //Intergral
  climberMotorConfig.kD = 0.000001; //Divirative 
  climberMotor->GetConfigurator().Apply(climberMotorConfig);
  climberMotor->SetInverted(false);
  climberMotor->SetNeutralMode(signals::NeutralModeValue::Brake);

  climberMotor->SetPosition(degree_t(0));
  SetName("Climber");
}

void Climber::SetClimberSpeed(double speed)
{
  climberMotor->Set(speed);
}

frc2::CommandPtr Climber::Ascend()
{
  return frc2::FunctionalCommand(
    [this] {Climber_Pawl_Sol->Set(frc::DoubleSolenoid::kForward);},
    [this] { SetClimberSpeed(1);},
    [this] (bool interrupted){SetClimberSpeed(0);},
    [this] {return false;},
    {this}
  ).ToPtr();
}

frc2::CommandPtr Climber::Descend()
{
  return frc2::FunctionalCommand(
    [this] {Climber_Pawl_Sol->Set(frc::DoubleSolenoid::kForward);},
    [this] { SetClimberSpeed(-0.8);},
    [this] (bool interrupted){SetClimberSpeed(0);},
    [this] {return false;},
    {this}
  ).ToPtr();
}

frc2::CommandPtr Climber::LockIn()
{
  return frc2::FunctionalCommand(
    [this] {Climber_Pawl_Sol->Set(frc::DoubleSolenoid::kReverse);},
    [this] {},
    [this] (bool interrupted){Climber_Pawl_Sol->Set(frc::DoubleSolenoid::kForward);},
    [this] {return false;},
    {this}
  ).ToPtr();
}

frc2::CommandPtr Climber::Test_Lock_Pawl()
{
  return this->RunOnce(
  [this] { Climber_Pawl_Sol->Set(frc::DoubleSolenoid::kForward); });
}

frc2::CommandPtr Climber::Test_Unlock_Pawl()
{
  return this->RunOnce(
  [this] { Climber_Pawl_Sol->Set(frc::DoubleSolenoid::kReverse); });
}