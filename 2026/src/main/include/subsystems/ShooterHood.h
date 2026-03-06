// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "generated/TunerConstants.h"
#include <frc2/command/Commands.h>
#include <frc/DoubleSolenoid.h>

class ShooterHood : public frc2::SubsystemBase 
{
 public:
  ShooterHood();
  
  auto Toggle() -> frc2::CommandPtr;
  auto Up() -> frc2::CommandPtr;
  auto Down() -> frc2::CommandPtr;

private:
  frc::DoubleSolenoid m_doubleSolenoid;
};
