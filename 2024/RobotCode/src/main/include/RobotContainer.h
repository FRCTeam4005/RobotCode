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
#include "subsystems/Arm.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Extender.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/LimeLight.h"
#include "subsystems/LimeLightData.h"
#include "subsystems/Climber.h"
#include "commands/SwerveControlCmd.h"
#include "commands/Extension.h"
#include "subsystems/Pneumatics.h"
#include <frc/XboxController.h>

#include "OperatorController.h"
#include "DriverController.h"

class RobotContainer {
 public:
  RobotContainer();
  frc2::CommandPtr GetAutonomousCommand(std::string&);
  auto SetCamToPigeon() -> void;
  // auto ShootOnceAuto() -> frc2::CommandPtr;
  auto ResetPose() -> void;
  frc2::CommandPtr StopDrive();

 private:

  const uint8_t pipeLineCnt{2};
  std::unique_ptr<LimeLightData> m_LimeLightData;
  std::unique_ptr<LimeLight> m_LimeLight;
  std::unique_ptr<Drivetrain> Drivetrain_Sys;
  std::unique_ptr<ctre::phoenix6::hardware::Pigeon2> Pigeon_Sys;

  std::unique_ptr<Arm> Arm_Sys;
  std::unique_ptr<Extension> Extensions_Cmds;
  std::unique_ptr<Pneumatics> Pneumatics_Sys;
  std::unique_ptr<Extender> Extender_Sys;
  std::unique_ptr<Climber> Climber_Sys;

  std::unique_ptr<Intake> Intake_Sys;
  std::unique_ptr<Shooter> Shooter_Sys;
  std::unique_ptr<frc::PowerDistribution> PDH;

  

  DriverController Driver{0};
  OperatorController Operator{1};

  void ConfigureBindings();
};
