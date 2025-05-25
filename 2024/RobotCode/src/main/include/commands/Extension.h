
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <subsystems/Extender.h>
#include "subsystems/Intake.h"
#include "subsystems/Arm.h"
#include "Constants.h"

using namespace CANConstants;

class Extension{
 public:


  auto extensionIn(Extender* Extension_Sys, Arm* Arm_Sys, Intake* Intake_Sys) -> frc2::CommandPtr;
  auto extensionOut(Extender* Extension_Sys, Arm* Arm_Sys, Intake* Intake_Sys) -> frc2::CommandPtr;
};