// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/smartdashboard/SendableChooser.h>
#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  std::optional<frc2::CommandPtr> m_autonomousCommand;
  frc::SendableChooser<std::string> m_chooser;
  const std::string CenterTwoPiece = "Center 2 Note";
  const std::string CenterThreeNote = "CenterAmp 3 Piece Auto";
  const std::string CenterThreeStage = "CenterStage 3 Piece Auto";
  const std::string OpenHoard = "Open Hoard Auto";
  const std::string AmpShoot = "Amp Shoot";
  const std::string Amp2Note = "Amp 2 Piece Auto";
  std::string m_autoSelected;

  RobotContainer m_container;
};
