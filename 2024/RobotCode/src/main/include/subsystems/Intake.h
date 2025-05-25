// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include "Constants.h"
#include <units/angle.h>
#include <frc2/command/Commands.h>
#include <frc2/command/TrapezoidProfileSubsystem.h>
#include <frc2/command/button/Trigger.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include "OperatorController.h"
#include "DriverController.h"
#include <iostream>


class Intake : public frc2::SubsystemBase {
 public:
  Intake(DriverController* Driver, OperatorController* Operator);
  
  auto FeedNoteToShooter() -> frc2::CommandPtr;
  auto GetNoteOffFloor() -> frc2::CommandPtr;
  auto Stop() -> frc2::CommandPtr;
  frc2::Trigger IsNoteInIntake();

 private:
  std::unique_ptr<rev::CANSparkMax> intakeMotor;
  void Periodic () override 
  {
    frc::SmartDashboard::PutBoolean("Note Sensor", IsNoteInIntake().Get());
  }
  frc::DigitalInput NoteSensor{OIConstants::NoteSensorPort};

  void setSpeed(double speed);
  DriverController* Driver;
  OperatorController* Operator;
};
