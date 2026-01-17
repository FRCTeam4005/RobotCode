// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>
#include <string>
#include <map>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/PowerDistribution.h>
#include <frc/DriverStation.h>

#include <ctre/phoenix6/Pigeon2.hpp>

#include "Constants.h"
#include "subsystems/Drivetrain.h"

#include <frc/XboxController.h>

#include "OperatorController.h"
#include "DriverController.h"

class RobotContainer {
 public:
  RobotContainer();
  frc2::CommandPtr GetAutonomousCommand();
  // auto ShootOnceAuto() -> frc2::CommandPtr;
  auto ResetPose() -> void;
  frc2::CommandPtr StopDrive();

 private:
  std::unique_ptr<Drivetrain> Drivetrain_Sys;
  std::unique_ptr<ctre::phoenix6::hardware::Pigeon2> Pigeon_Sys;
  std::unique_ptr<frc::PowerDistribution> PDH;

  

  DriverController Driver{0};
  OperatorController Operator{1};

  void ConfigureBindings();
};
