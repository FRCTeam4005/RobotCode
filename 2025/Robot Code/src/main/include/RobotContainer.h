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
#include "subsystems/Algae.h"
#include "subsystems/Claw.h"
#include "subsystems/Elevator.h"
#include "subsystems/Algae.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include "commands/Commands.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include "Constants.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Algae.h"
#include <frc/XboxController.h>
#include "subsystems/Winch.h"
#include "subsystems/ServoClamp.h"
#include "subsystems/CANdi.h"
#include "subsystems/CANdleSystem.h"

#include "OperatorController.h"
#include "DriverController.h"

class RobotContainer {
 public:
  RobotContainer();
  frc2::CommandPtr GetAutonomousCommand(std::string pathPlannerAuto) ;
  auto ResetPose() -> void;
  auto ScanCANdi() -> void;
  frc2::CommandPtr StopDrive();
  auto autoUpdate() -> void;
  void ResetPigeonToCamera();

 private:
  std::unique_ptr<Claw> Claw_Sys;
  std::unique_ptr<Elevator> Elevate_Sys;
  std::unique_ptr<Scoring> Command_Sys;
  std::unique_ptr<Drivetrain> Drivetrain_Sys;
  std::unique_ptr<ctre::phoenix6::hardware::Pigeon2> Pigeon_Sys;
  std::unique_ptr<frc::PowerDistribution> PDH;
  std::unique_ptr<Algae> Algae_Sys;
  std::unique_ptr<CANDigitalInput> ClawElevatorCANdi;
  std::unique_ptr<Winch> Winch_Sys;
  std::unique_ptr<WinchServo> Servo_Sys;
  std::unique_ptr<CANdleSystem> CANdle_Sys;


  DriverController Driver{0};
  OperatorController Operator{1};

  void DriverCommands();
  void OperatorCommands();
  void AutomaticCommands();
  void AutoNamedCommands();
  void ConfigureBindings();
};
