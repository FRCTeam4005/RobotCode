// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "LimelightHelpers.h"

#include <frc2/command/CommandScheduler.h>


constexpr auto LimeLightYawOffset = 180_deg;

Robot::Robot() {}

void Robot::RobotPeriodic() {
    m_timeAndJoystickReplay.Update();
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() 
{

    if (BodyTargetAvaliable())
    {
        //the internal IMU just sets the megatag2 yaw to 0 on start so we yoink it from megatag 1 since megatag one does know the yaw but is just not stable most of the time
        auto UnstableYaw = LimelightHelpers::getBotPose2d_wpiBlue("limelight-bodycam").Rotation().Degrees().value();
        LimelightHelpers::SetRobotOrientation("limelight-bodycam",UnstableYaw,0,0,0,0,0);

        //the internal IMU just sets the megatag2 yaw to 0 on start so we yoink it from megatag 1 since megatag one does know the yaw but is just not stable most of the time
        TurretCameraPose = LimelightHelpers::getBotPose2d_wpiBlue("limelight-turret").Rotation().Degrees().value();
        LimelightHelpers::SetRobotOrientation("limelight-turret",UnstableYaw,0,0,0,0,0);

        //set the pose of the drive train pose to match with the 
        m_container.drivetrain.ResetPose(BodyGetPose());

        m_container.Turret_Sys->CalibratePose();
    }
    


}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {

    auto BotPose = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight-bodycam").pose;
    BotPose = frc::Pose2d{BotPose.X(), BotPose.Y(), frc::Rotation2d{BotPose.Rotation().Degrees()}};

    m_container.drivetrain.ResetPose(BotPose);
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand.value());
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand.value());
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
