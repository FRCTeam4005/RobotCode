// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/HootAutoReplay.hpp"

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <optional>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
public:
    Robot();
    void RobotPeriodic() override;
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
    static constexpr bool kUseLimelight = false;

    frc2::Command *m_autonomousCommand;
    frc::SendableChooser<std::string> m_chooser;

    RobotContainer m_container;

    /* log and replay timestamp and joystick data */
    ctre::phoenix6::HootAutoReplay m_timeAndJoystickReplay = ctre::phoenix6::HootAutoReplay{}
        .WithTimestampReplay()
        .WithJoystickReplay();
    
    std::vector<std::string> AvaliablePathPlannerAutos =
    {
        "Left Auto",
        "Right Auto",
        "New Auto"
    };




    frc::Pose2d getAlliancePose(std::string CameraName)
    {
        frc::Pose2d CameraPose;


        if (auto ally = frc::DriverStation::GetAlliance()) 
        {
            if (ally.value() == frc::DriverStation::Alliance::kRed) 
            {
                CameraPose = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(CameraName).pose;
            }
            if (ally.value() == frc::DriverStation::Alliance::kBlue) {
                CameraPose = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(CameraName).pose;
            }
        }
        else 
        {
        }

        frc::Pose2d BotPose = frc::Pose2d{CameraPose.X(), CameraPose.Y(), frc::Rotation2d{CameraPose.Rotation().Degrees()}};
        return BotPose;
    }

    bool BodyTargetAvaliable()
    {
        return LimelightHelpers::getTV("limelight-bodycam") > 0;
    }

    frc::Pose2d BodyGetPose()
    {
        return getAlliancePose("limelight-bodycam");
    }

};
