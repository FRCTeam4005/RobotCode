
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "generated/TunerConstants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <units/angle.h>
#include <units/voltage.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>
#include <iostream>

class Intake : public frc2::SubsystemBase
{
 public:
  Intake ();
  
private:
  

};