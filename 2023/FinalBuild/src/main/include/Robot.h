// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "Drivetrain.h"
#include "DriverController.h"
#include "OperatorController.h"
#include "FourBarArm.h"
#include <memory>
#include <units/length.h>
#include "Pigeon.h"

#include "Drivetrain.h"
#include "DriverController.h"
#include "OperatorController.h"
#include "FourBarArm.h"
#include <memory>
#include <units/length.h>
#include "Pigeon.h"
#include "RGBStrip.h"
#include "LimeLightVision.h"
#include "auto/Drive.h"
#include "auto/WheelAlign.h"
#include "auto/FourBar.h"
#include "auto/Balance.h"
#include "auto/LevelBase.h"
#include "auto/OverChargeStn.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  
 private:
  typedef enum {
    PLACE_OBJECT,
    DRIVE,
    DONE
  } AutoStates;

  AutoStates CurrentState = PLACE_OBJECT;

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoConeDrive = "Cone and Drive";
  const std::string kAutoConeBalance = "Cone and Balance";
  const std::string kAutoConeDriveBalance = "Cone, Drive, and Balance";
  std::string m_autoSelected;
  FourBarArm _FourBarArm;

  RGBStrip ColorStrip;
  LimeLightVision RobotLimeLight;

  CameraDistance TargetDistance;
};
