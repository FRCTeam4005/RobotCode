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
    

    m_chooser.SetDefaultOption(AvaliablePathPlannerAutos[1],AvaliablePathPlannerAutos[1]);
    for(auto Path : AvaliablePathPlannerAutos)
    {
        m_chooser.AddOption(Path, Path);
    }
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {
        auto const driveState = m_container.drivetrain.GetState();
        auto const heading = driveState.Pose.Rotation().Degrees();
        auto const omega = driveState.Speeds.omega;

        LimelightHelpers::SetRobotOrientation("limelight", heading.value(), 0, 0, 0, 0, 0);
        auto llMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (llMeasurement.tagCount > 0 && units::math::abs(omega) < 2_tps) {
            m_container.drivetrain.AddVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }
    }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() 
{}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {

    auto BotPose = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight-bodycam").pose;
    BotPose = frc::Pose2d{BotPose.X(), BotPose.Y(), frc::Rotation2d{BotPose.Rotation().Degrees()}};

    m_container.drivetrain.ResetPose(BotPose);

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
    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Cancel(std::move(m_autonomousCommand.value()));
    }
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
