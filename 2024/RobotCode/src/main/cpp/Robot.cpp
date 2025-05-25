// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Drivetrain.h"

void Robot::RobotInit() 
{
  const std::string CenterTwoPiece = "Center 2 Note";
  const std::string CenterThreeNote = "CenterAmp 3 Piece Auto";
  const std::string CenterThreeStage = "CenterStage 3 Piece Auto";
  // LimelightHelpers::SetupPortForwarding("limelight");
  m_chooser.SetDefaultOption("Center 2 Note", CenterTwoPiece);
  m_chooser.AddOption("Center/Amp 3 Note", CenterThreeNote);
  m_chooser.AddOption("Center/Stage 3 Note", CenterThreeStage);
  m_chooser.AddOption("Open Side Hoard", OpenHoard);
  m_chooser.AddOption("Amp Shoot", AmpShoot);
  m_chooser.AddOption("Amp 2 Note", Amp2Note);
  m_chooser.AddOption("Amp 1 Note Auto", AmpShoot);
  m_chooser.AddOption("Nothing", "Nothing");
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  m_container.ResetPose();
  frc::SmartDashboard::PutNumber("omega", 0);
  frc::SmartDashboard::PutNumber("TargetX", 0);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() 
{
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit()
{
  m_container.ResetPose();
  m_container.SetCamToPigeon();

  m_autoSelected = m_chooser.GetSelected();
  frc::SmartDashboard::PutString("Auto Mode Selected", m_autoSelected);

  m_autonomousCommand = m_container.GetAutonomousCommand(m_autoSelected);

  if(m_autonomousCommand) 
  {
    m_autonomousCommand->Schedule();
  }
  
}

void Robot::AutonomousPeriodic() 
{

}

void Robot::TeleopInit() 
{
  m_container.ResetPose();
  m_container.SetCamToPigeon();

  if (m_autonomousCommand) 
  {
    m_autonomousCommand->Cancel();
    m_container.StopDrive();
  }
  
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
  