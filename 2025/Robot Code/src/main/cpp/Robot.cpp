// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <cameraserver/CameraServer.h>

#include <frc/DriverStation.h>
#include "LimeLightHelpers.h"

Robot::Robot() 
: m_container{std::make_unique<RobotContainer>()}
{
  frc::CameraServer::StartAutomaticCapture();
  m_container->ResetPigeonToCamera();
    m_chooser.SetDefaultOption(AvaliablePathPlannerAutos[1],AvaliablePathPlannerAutos[1]);
  for(auto Path : AvaliablePathPlannerAutos)
  {
    m_chooser.AddOption(Path, Path);

  }

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::RobotInit()
{
  m_container->ResetPose();
}

uint8_t paceVar = 0;
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();


    m_container->ScanCANdi();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() 
{
  using namespace LimelightHelpers;

  
  
  getTV("limelight") ? setLEDMode_ForceOff("limelight") : setLEDMode_ForceOn("limelight");
}

void Robot::DisabledExit() 
{
  LimelightHelpers::setLEDMode_ForceOff("limelight");
  // m_container->ResetPose(); 
}

void Robot::AutonomousInit() {

  m_container->ResetPigeonToCamera();
  m_autonomousCommand = m_container->GetAutonomousCommand(m_chooser.GetSelected());

  if(m_autonomousCommand) 
  {
    m_autonomousCommand->Schedule();
  }

}

void Robot::AutonomousPeriodic() {
  m_container->autoUpdate();
}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  // m_container->ResetPigeonToCamera();
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
  
}

void Robot::TeleopPeriodic() {
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
