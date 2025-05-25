// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <string>

#include "RobotContainer.h"
#include "subsystems/CANdleSystem.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void RobotInit() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

 private:
  std::optional<frc2::CommandPtr> m_autonomousCommand;
  frc::SendableChooser<std::string> m_chooser;
  std::string m_autoSelected;

  std::unique_ptr<RobotContainer> m_container;

  std::vector<std::string> AvaliablePathPlannerAutos =
  {
    "Left Auto",
    "Right Auto",
    "Middle"
  };
};
