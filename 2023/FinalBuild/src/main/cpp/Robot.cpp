// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include "iterator.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <list>
#include <units/time.h>

void Robot::RobotInit() 
{
  m_chooser.SetDefaultOption(kAutoConeDrive, kAutoConeDrive);
  m_chooser.AddOption(kAutoConeBalance, kAutoConeBalance);
  m_chooser.AddOption(kAutoConeDriveBalance, kAutoConeDriveBalance);
  frc::SmartDashboard::PutNumber("POSITION P", 1.4);
  frc::SmartDashboard::PutNumber("POSITION I", 0);
  frc::SmartDashboard::PutNumber("POSITION D", 0.0032);
  frc::SmartDashboard::PutNumber("SetPoint", 180);
}

void Robot::RobotPeriodic() 
{}

iterator RobotAutoSteps{};

void Robot::AutonomousInit() 
{
  RobotAutoSteps.clear();
  m_autoSelected = m_chooser.GetSelected();

  RobotAutoSteps.clear();
  RobotAutoSteps.addStep(std::make_unique<RobotDriveAlign>(Drivetrain::getInstance()));
  RobotAutoSteps.addStep(std::make_unique<RobotAutoMech>(_FourBarArm));

  if (m_autoSelected == kAutoConeDrive) 
  {
    //RobotAutoSteps.addStep(std::make_unique<RobotAutoDrive>(12_ft, 0_ft, 0_deg, 1.5_mps, Drivetrain::getInstance()));
    RobotAutoSteps.addStep(std::make_unique<RobotAutoDrive>(12_ft, 0_ft, 90_deg, 1.5_mps, Drivetrain::getInstance()));
  } 
  else if(m_autoSelected == kAutoConeBalance) 
  {
    RobotAutoSteps.addStep(std::make_unique<RobotAutoBalance>(2.5_fps, 1_fps, RobotLeveling::RobotDirection::TowardOpposingTeam, Drivetrain::getInstance()));
  }
  else if(m_autoSelected == kAutoConeDriveBalance)
  {
    RobotAutoSteps.addStep(std::make_unique<RobotOverChargeStation>(2.5_fps, 1_fps, RobotLeveling::RobotDirection::TowardOpposingTeam, Drivetrain::getInstance()));
  }

  RobotAutoSteps.Init();
}

void Robot::AutonomousPeriodic() 
{
  RobotAutoSteps.Run();
}

void Robot::TeleopInit()
{
  _FourBarArm.TeleopInit();
}

void Robot::TeleopPeriodic() 
{
  _FourBarArm.UpDown(OperatorController::GetInstance());
  _FourBarArm.InOut(OperatorController::GetInstance());
  _FourBarArm.Intake(OperatorController::GetInstance());
  ColorStrip.RGB_Color_Toggle(DriverController::GetInstance());
  _FourBarArm.MovementPresets(OperatorController::GetInstance());
  Drivetrain::getInstance().teleOpSwerve(DriverController::GetInstance());
}

void Robot::DisabledInit() 
{}

void Robot::DisabledPeriodic() 
{}

frc::PIDController testRCWPos{0,0,0};
void Robot::TestInit() 
{
  testRCWPos.EnableContinuousInput(0,360);
  testRCWPos.SetTolerance(5,5);
}


void Robot::TestPeriodic() 
{
  testRCWPos.SetP(frc::SmartDashboard::GetNumber("POSITION P", 1.4));
  testRCWPos.SetI(frc::SmartDashboard::GetNumber("POSITION I", 0));
  testRCWPos.SetD(frc::SmartDashboard::GetNumber("POSITION D", 0.0032));
  testRCWPos.SetSetpoint(frc::SmartDashboard::GetNumber("SetPoint", 180));
  frc::SmartDashboard::PutNumber("position of robo", testRCWPos.GetPositionError());
  
  auto PIDrcwValue = units::degrees_per_second_t{testRCWPos.Calculate(Pigeon::GetInstance().GetPigeonData().FusionAngle.value() + 180)};


  if(testRCWPos.AtSetpoint())
  {
    Drivetrain::getInstance().fieldCentricSwerveCalculation(DriverController::GetInstance().getFWD(), DriverController::GetInstance().getSTR(), 0_deg_per_s);
  }
  else
  {
    Drivetrain::getInstance().fieldCentricSwerveCalculation(DriverController::GetInstance().getFWD(), DriverController::GetInstance().getSTR(), PIDrcwValue);
  }
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
