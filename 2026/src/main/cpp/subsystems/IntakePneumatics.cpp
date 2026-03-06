#include "subsystems/IntakePneumatics.h"

#define INTAKE_IN frc::DoubleSolenoid::Value::kForward
#define INTAKE_OUT frc::DoubleSolenoid::Value::kReverse


IntakePneumatics::IntakePneumatics()
:m_doubleSolenoid( CANConstants::kPneumaticHub,
                   frc::PneumaticsModuleType::REVPH,
                   PneumaticsChannelConst::kIntakeOutChannel,
                   PneumaticsChannelConst::kIntakeInChannel )
{
  m_doubleSolenoid.Set(INTAKE_IN); 
  SetName("IntakePneumatics");
}

frc2::CommandPtr IntakePneumatics::Toggle()
{
  return this->RunOnce(
    [this] {m_doubleSolenoid.Toggle(); }
  );

}
frc2::CommandPtr IntakePneumatics::In()
{
  return this->RunOnce(
    [this] {m_doubleSolenoid.Set(INTAKE_IN); }
  );

}
frc2::CommandPtr IntakePneumatics::Out()
{
  return this->RunOnce(
    [this] {m_doubleSolenoid.Set(INTAKE_OUT); }
  );

}