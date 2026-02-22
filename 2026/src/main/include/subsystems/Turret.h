
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

class Turret : public frc2::SubsystemBase
{
 public:
    Turret();
    auto Move(units::turn_t goal) -> frc2::CommandPtr;
    auto ShootDrivers() -> frc2::CommandPtr;
    auto GetPosition() -> units::angle::turn_t;
    auto TrackTag(std::function<frc::Pose2d()> getRobotPose) -> frc2::CommandPtr;
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
    std::unique_ptr<frc::PIDController> turret_controller;
    double feedforward;
    frc::Pose2d m_TurretCameraPose;

    

    void Periodic () override
    {


        position = GetPosition();

        //just update the camera pose once to reduce blocking calls
        m_TurretCameraPose = TurretGetPose();

        // tx = LimelightHelpers::getTX("limelight-turret");
        // frc::SmartDashboard::PutNumber("tx camera offset", tx);
        // target = LimelightHelpers::getTV("limelight-turret");
        // frc::SmartDashboard::PutBoolean("Target Detected", target);
        
        // units::meter_t desiredX = 4.625_m;
        // units::meter_t desiredY = 4.030_m;

        // auto currentPose = m_getPose();
        // if (target)
        // {
        // frc::SmartDashboard::PutNumber("Desired X", double(desiredX));
        // frc::SmartDashboard::PutNumber("Desired Y", double(desiredY));
        // frc::SmartDashboard::PutNumber("Current Y", currentPose.Y().value());
        // frc::SmartDashboard::PutNumber("Current X", currentPose.X().value());

        // frc::SmartDashboard::PutNumber("Delta Y", double(double(desiredY) - currentPose.Y().value()));
        // frc::SmartDashboard::PutNumber("Delta X", double(double(desiredX) - currentPose.X().value()));

        

        // auto Theta = (atan((desiredY.value() - currentPose.Y().value()) / (desiredX.value() - currentPose.X().value()))*180)/3.14;
        // angle = ((Theta - Pigeon_Sys->GetYaw().GetValueAsDouble())/360.0);
        // frc::SmartDashboard::PutNumber("Angle", angle);


        // auto desiredOutput = turret_controller->Calculate(currentPose.Rotation().Degrees().value(), units::radian_t(Theta).convert<units::degree>().value());


        // frc::SmartDashboard::PutNumber("current degrees", currentPose.Rotation().Degrees().value());
        // //frc::SmartDashboard::PutNumber("Desired Degrees", units::radian_t(Theta).convert<units::degree>().value());
        // frc::SmartDashboard::PutNumber("Desired Degrees", Theta);
        // frc::SmartDashboard::PutNumber("Turret COntroll Output", desiredOutput);
        // }
    }

    void SetTurretCommand(units::turn_t goal);
    
    void Track(std::function<frc::Pose2d()> getRobotPose);
    void Stop();


    frc::Pose2d getAlliancePose(std::string CameraName)
    {
        frc::Pose2d CameraPose;


        if (auto ally = frc::DriverStation::GetAlliance()) 
        {
            if (ally.value() == frc::DriverStation::Alliance::kRed) 
            {
                CameraPose = LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2(CameraName).pose;
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