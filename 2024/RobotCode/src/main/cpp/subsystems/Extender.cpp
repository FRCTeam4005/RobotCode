#include "subsystems/Extender.h"
#include "frc2/command/FunctionalCommand.h"
#include "subsystems/Arm.h"


Extender::Extender(Pneumatics *Pneumatics_Sys)
{
  Extender_Sol = std::make_unique<frc::DoubleSolenoid>(Pneumatics_Sys->GetDoubleSolenoid(0,1));
  
}

void Extender::InitSendable(wpi::SendableBuilder& builder)
{
  // builder.AddDoubleProperty{}
}

frc2::CommandPtr Extender::SetPastBumper()
{
  return this->RunOnce(
  [this] { Extender_Sol->Set(frc::DoubleSolenoid::kForward); });
}

frc2::CommandPtr Extender::SetIntoFrame()
{
  return this->RunOnce(
  [this] { Extender_Sol->Set(frc::DoubleSolenoid::kReverse); });
}