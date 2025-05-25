#include "frc2/command/FunctionalCommand.h"
#include "commands/Commands.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/WaitCommand.h>

using namespace frc2;

//Moves the elevator to L2
frc2::CommandPtr Scoring::L2(Elevator* Elevate_Sys, Claw* Claw_Sys)
{
  return Claw_Sys->SetPosition(ClawPosition::PATH).AlongWith(std::move(Elevate_Sys->SetToLevel(Level::L2)));
}

//Moves the elevator to L3
frc2::CommandPtr Scoring::L3(Elevator* Elevate_Sys, Claw* Claw_Sys)
{
  return Claw_Sys->SetPosition(ClawPosition::PATH).AlongWith(std::move(Elevate_Sys->SetToLevel(Level::L3)));
}

//Moves the elevator to L4
frc2::CommandPtr Scoring::L4(Elevator* Elevate_Sys, Claw* Claw_Sys)
{
  return Claw_Sys->SetPosition(ClawPosition::PATH).AlongWith(std::move(Elevate_Sys->SetToLevel(Level::L4)));
}

frc2::CommandPtr Scoring::AutoL4(Elevator* Elevate_Sys, Claw* Claw_Sys)
{
  return Elevate_Sys->SetToLevel(Level::L4)
        //Change this to adjust auto claw timing
        .AlongWith(std::move(WaitCommand(units::time::second_t(.25)))
        .AndThen(std::move(Claw_Sys->SetPosition(ClawPosition::PATH))));
}

frc2::CommandPtr Scoring::Score(Elevator* Elevate_Sys, Claw* Claw_Sys)
{
  return Claw_Sys->SetPosition(ClawPosition::SCORE);
}

//Sets the elevator and claw to ready/collect
frc2::CommandPtr Scoring::Ready(Elevator* Elevate_Sys, Claw* Claw_Sys)
{
  return Elevate_Sys->SetToLevel(Level::READY).AlongWith(std::move(Claw_Sys->SetToCollect()));
}

frc2::CommandPtr Scoring::Collect(Elevator* Elevate_Sys, Claw* Claw_Sys)
{
  return Elevate_Sys->SetToCollect()
         .AndThen(std::move(WaitCommand(units::second_t(.4))).ToPtr())
         .AndThen((std::move(Elevate_Sys->SetToLevel(Level::L3)))
         //Change this to adjust claw timing
         .AlongWith((std::move(WaitCommand(units::second_t(.3))).ToPtr())
         .AndThen(std::move(Claw_Sys->SetPosition(ClawPosition::PATH)))));
}

frc2::CommandPtr Scoring::AutoCollect(Elevator* Elevate_Sys, Claw* Claw_Sys)
{
  return Elevate_Sys->SetToCollect()
  .AndThen(std::move(WaitCommand(units::second_t(.4))).ToPtr());
}
