#include "subsystems/Shooter/Hood.h"

#define HOOD_UP frc::DoubleSolenoid::Value::kForward
#define HOOD_DOWN frc::DoubleSolenoid::Value::kReverse

ShooterHood::ShooterHood()
:m_doubleSolenoid( CANConstants::kPneumaticHub,
                   frc::PneumaticsModuleType::REVPH,
                   PneumaticsChannelConst::kShooterDownChannel,
                   PneumaticsChannelConst::kShooterUpChannel )
{
  m_doubleSolenoid.Set(HOOD_DOWN);
  SetName("ShooterHood");
}

frc2::CommandPtr ShooterHood::Toggle()
{
  return this->RunOnce(
    [this] {m_doubleSolenoid.Toggle(); }
  );

}
frc2::CommandPtr ShooterHood::Up()
{
  return this->RunOnce(
    [this] {m_doubleSolenoid.Set(HOOD_UP); }
  );

}

frc2::CommandPtr ShooterHood::Down()
{
  return this->RunOnce(
    [this] {m_doubleSolenoid.Set(HOOD_DOWN); }
  );

}
