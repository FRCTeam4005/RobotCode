// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot() 
{
}

void Robot::RobotPeriodic() {
    
    m_timeAndJoystickReplay.Update();
    frc2::CommandScheduler::GetInstance().Run();
    

    m_chooser.SetDefaultOption(AvaliablePathPlannerAutos[0],AvaliablePathPlannerAutos[0]);
    for(auto Path : AvaliablePathPlannerAutos)
    {
        m_chooser.AddOption(Path, Path);
    }
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() 
{}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {

    this->SetRobotAutoRoutine(m_chooser.GetSelected());
    m_autonomousCommand = this->GetRobotAutoCommand();


    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Schedule(std::move(m_autonomousCommand.value()));
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() 
{
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TeleopInit() {
    // if (m_autonomousCommand) {
    //     frc2::CommandScheduler::GetInstance().Cancel(std::move(m_autonomousCommand.value()));
    // }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    // frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
