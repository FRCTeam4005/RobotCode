
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "generated/TunerConstants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <units/angle.h>
#include <units/voltage.h>
#include <units/angle.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>
#include <iostream>
#include "frc2/command/FunctionalCommand.h"
#include "frc/smartdashboard//SmartDashboard.h"
#include <cmath>
#include "subsystems/Drivetrain.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/controller/PIDController.h>
#include "LimelightHelpers.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <Optional>
#include <functional>

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define BLUE_LINE_COORD 4.625594_m
#define RED_LINE_COORD  11.888_m
#define MID_FIELD_LINE 4.0211375_m


class Turret : public frc2::SubsystemBase
{
 public:
    Turret(std::function<frc::Pose2d()> getRobotPose, std::function<void(frc::Pose2d, units::time::second_t)> setVisionMeasurement);
    auto Move(units::turn_t goal) -> frc2::CommandPtr;
    auto ShootDrivers() -> frc2::CommandPtr;
    auto getTurretPosition() -> units::angle::turn_t;
    auto TrackTag() -> frc2::CommandPtr;
    auto StopTrackingTag() -> frc2::CommandPtr;
    auto m_getPose() -> frc::Pose2d;
    auto CalibratePose() -> void;

private:
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> TurretMotor;
    subsystems::Drivetrain drivetrain{TunerConstants::CreateDrivetrain()};
    std::unique_ptr<ctre::phoenix6::hardware::Pigeon2> Pigeon_Sys;
    ctre::phoenix6::controls::MotionMagicVoltage elevate_mmReq{0_tr};
    
    
    
    units::turn_t position;
    double angle;
    double tx;
    bool target;
    double omega;
    std::unique_ptr<frc::PIDController> turret_controller;
    double feedforward = 0.025;
    
    frc::Field2d m_field;
    frc::Field2d m_DesiredPoseField;
    frc::Pose2d m_TurretCameraPose;
    frc::Pose2d m_TurretPose;
    frc::Pose2d m_RobotPose;
    
    double m_Theta;
    
    const frc::Translation2d SauronBlue = frc::Translation2d(BLUE_LINE_COORD, 4.034663_m);
    const frc::Translation2d LeftPassBlue = frc::Translation2d(1_m, 1.5_m);
    const frc::Translation2d RightPassBlue = frc::Translation2d(1_m, 7_m);
    
    const frc::Translation2d SauronRed = frc::Translation2d(11.888_m, 4.034663_m);
    const frc::Translation2d LeftPassRed = frc::Translation2d(15.5_m, 1.5_m);
    const frc::Translation2d RightPassRed = frc::Translation2d(15.5_m, 7_m);
    
    
    std::function<void(frc::Pose2d, units::time::second_t)> setRobotBodyVisionMeasurement;
    std::function<frc::Pose2d()> getRobotBodyPose;
    void Periodic () override;
    auto Track()  -> void;
    auto Stop() -> void;
    auto CalculateTheta(frc::Translation2d TargetPose) -> void;
    auto SetTurretCommand(units::turn_t goal) -> void;
    auto GetPosition() -> units::turn_t;




    frc::Pose2d getAlliancePose(std::string CameraName)
    {
        frc::Pose2d CameraPose;
        CameraPose = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(CameraName).pose;

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