
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "generated/TunerConstants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <frc/DoubleSolenoid.h>


class IntakePneumatics : public frc2::SubsystemBase
{
 public:
  IntakePneumatics ();
  auto Toggle() -> frc2::CommandPtr;
  auto In() -> frc2::CommandPtr;
  auto Out() -> frc2::CommandPtr;
  
private:
  frc::DoubleSolenoid m_doubleSolenoid;
};