// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/Drivetrain.h"
#include "Telemetry.h"
#include "subsystems/Turret.h"
#include "subsystems/ShooterHood.h"
#include "subsystems/ShooterKicker.h"
#include "subsystems/ShooterWheels.h"
#include "subsystems/Compressor.h"
#include "subsystems/IntakeConveyor.h"
#include "subsystems/IntakeFrontRoller.h"

class RobotContainer {

    
public:
    subsystems::Drivetrain drivetrain{TunerConstants::CreateDrivetrain()};
    std::unique_ptr<Turret> Turret_Sys;
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();
    void CalibrateSensors();

private:
    Compressor Compressor_Sys;
    
    std::unique_ptr<ShooterHood> ShooterHood_Sys;
    std::unique_ptr<ShooterKicker> ShooterKicker_Sys;
    std::unique_ptr<ShooterWheels> ShooterWheels_Sys;
    std::unique_ptr<IntakeConveyor> IntakeConveyor_Sys;
    std::unique_ptr<IntakeFrontRoller> IntakeFrontRoller_Sys;
    
    void ConfigureBindings();
    void DriverControls();
    void OperatorControls();

    units::meters_per_second_t MaxSpeed = 0.5 * TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.05).WithRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{MaxSpeed};

    frc2::CommandXboxController Driver{0};
    frc2::CommandXboxController Operator{1};

    double angle;
    frc::SendableChooser<frc2::Command *> autoChooser;


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

    bool TurretTargetAvaliable()
    {
        return LimelightHelpers::getTV("limelight-turret") > 0;
    }

    //please just use m_TurretPose to reduce blocking calls to the network table
    frc::Pose2d TurretGetPose()
    {   
        return getAlliancePose("limelight-turret");
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
