
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Elevator.h"
#include "subsystems/Claw.h"
#include "subsystems/Algae.h"
#include "Constants.h"

class Scoring{
 public:
  auto L2(Elevator* Elevate_Sys, Claw* Claw_Sys) -> frc2::CommandPtr;
  auto L3(Elevator* Elevate_Sys, Claw* Claw_Sys) -> frc2::CommandPtr;
  auto L4(Elevator* Elevate_Sys, Claw* Claw_Sys) -> frc2::CommandPtr;
  auto AutoL4(Elevator* Elevate_Sys, Claw* Claw_Sys) -> frc2::CommandPtr;
  auto Score(Elevator* Elevate_Sys, Claw* Claw_Sys) -> frc2::CommandPtr;
  auto Ready(Elevator* Elevate_Sys, Claw* Claw_Sys) -> frc2::CommandPtr;
  auto Collect(Elevator* Elevate_Sys, Claw* Claw_Sys) -> frc2::CommandPtr;
  auto AutoCollect(Elevator* Elevate_Sys, Claw* Claw_Sys) -> frc2::CommandPtr;

};