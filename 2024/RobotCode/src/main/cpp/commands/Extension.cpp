#include "subsystems/Extender.h"
#include "frc2/command/FunctionalCommand.h"
#include "subsystems/Arm.h"
#include "commands/Extension.h"
#include <frc2/command/WaitCommand.h>
#include <frc2/command/CommandPtr.h>

using namespace frc2;


frc2::CommandPtr Extension::extensionIn(Extender* Extension_Sys, Arm* Arm_Sys, Intake* Intake_Sys)
{
  if(!Extension_Sys)
  {
    return frc2::cmd::Print("extensionIn: NullPtr Extension_Sys error!!!!");
  }

  if(!Arm_Sys)
  {
    return frc2::cmd::Print("extensionIn: NullPtr Arm_Sys error!!!!");
  }

  if(!Intake_Sys)
  {
    return frc2::cmd::Print("extensionIn: NullPtr Intake_Sys error!!!!");
  }

  return Arm_Sys->SetAboveBumperPosition().AndThen(Extension_Sys->SetIntoFrame()).AndThen(std::move(WaitCommand((units::second_t) 0.4)).ToPtr()).AndThen(Arm_Sys->SetOnSwervePosition()).AlongWith(Intake_Sys->Stop());
}

frc2::CommandPtr Extension::extensionOut(Extender* Extension_Sys, Arm* Arm_Sys, Intake* Intake_Sys)
{
  if(!Extension_Sys)
  {
    return frc2::cmd::Print("extensionOut: NullPtr Extension_Sys error!!!!");
  }

  if(!Arm_Sys)
  {
    return frc2::cmd::Print("extensionOut: NullPtr Arm_Sys error!!!!");
  }

  if(!Intake_Sys)
  {
    return frc2::cmd::Print("extensionOut: NullPtr Intake_Sys error!!!!");
  }

  return Arm_Sys->SetDesiredPosition(2_tr).AndThen(Extension_Sys->SetPastBumper()).AndThen(std::move(WaitCommand((units::second_t) .4)).ToPtr()).AndThen(Arm_Sys->SetBelowBumperPosition()).AndThen(Intake_Sys->GetNoteOffFloor());
}