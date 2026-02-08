#include "subsystems/Intake.h"


Intake::Intake()
:m_doubleSolenoid(CANConstants::kPneumaticHub,frc::PneumaticsModuleType::REVPH,0,1)
{
  IntakeMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kIntakeMotorID);
  ConveyorMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kConveyorMotorID);
  m_doubleSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  IntakeMotor->Set(0);
  ConveyorMotor->Set(0);
    SetName("Intake");
}

void Intake::setSpeed(double speed)
{
    IntakeMotor->Set(-speed);
    ConveyorMotor->Set(speed/2);
}

frc2::CommandPtr Intake::FuelOut()
{
    return frc2::FunctionalCommand(
        [this] {},
        [this] {setSpeed(-1);},
        [this] (bool interrupted) {setSpeed(0);},
        [this] {return true;},
        {this}
    ).ToPtr();
}

frc2::CommandPtr Intake::FuelUp()
{
  return frc2::FunctionalCommand(
    [this] {},
    [this] {setSpeed(.5);},
    [this] (bool interrupted) {setSpeed(0);},
    [this] {return false;},
    {this}
  ).ToPtr();
}

frc2::CommandPtr Intake::Stop()
{
  return this->RunOnce(
    [this] {setSpeed(0);}
  );
}

frc2::CommandPtr Intake::IntakeOut()
{
  return this->RunOnce(
    [this] {m_doubleSolenoid.Toggle(); }
  );

}
frc2::CommandPtr Intake::IntakeIn()
{
  return this->RunOnce(
    [this] {m_doubleSolenoid.Set(frc::DoubleSolenoid::Value::kReverse); }
  );

}