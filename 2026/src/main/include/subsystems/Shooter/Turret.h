
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
#include "lib/LimelightHelpers.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <Optional>
#include <functional>

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "AM_CAN_Mag_Switch.h"

#define BLUE_LINE_COORD 4.625594_m
#define RED_LINE_COORD  12.563094_m
#define MID_FIELD_LINE 4.0211375_m


class Turret : public frc2::SubsystemBase
{
 public:
    // Turret();
    Turret(std::function<frc::Pose2d()> getBotPose, std::function<units::angle::degree_t()> getIMU);
    auto Move(units::turn_t goal) -> frc2::CommandPtr;
    //auto ShootDrivers() -> frc2::CommandPtr;
    auto getTurretPosition() -> units::angle::turn_t;
    auto TrackTag() -> frc2::CommandPtr;
    auto StopTrackingTag() -> frc2::CommandPtr;
    auto m_getPose() -> frc::Pose2d;
    auto CalibratePose() -> void;
    auto GetDistanceMeters() -> double;
    bool IsHoodUp();
    auto ToggleTracking() -> frc2::CommandPtr;


private:
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> _Motor;
    AM_CAN_Mag_Switch RightMagSwtich{CANConstants::RightMagSwtich};
    AM_CAN_Mag_Switch MiddleMagSwitch{CANConstants::MiddleMagSwitch};
    AM_CAN_Mag_Switch LeftMagSwitch{CANConstants::LeftMagSwitch};    

private:


    ctre::phoenix6::controls::MotionMagicVoltage elevate_mmReq{0_tr};

    
    bool TurretTrack_{false};

    units::turn_t goal = 0_tr;
    
    units::turn_t position;
    double distance;
    double angle_;
    double tx;
    bool target;
    double omega;
    bool hoodUp;
    
    frc::Field2d m_field;
    frc::Field2d m_DesiredPoseField;
    frc::Pose2d m_TurretCameraPose;
    frc::Pose2d m_TurretPose;
    frc::Pose2d m_RobotPose;
    
    double m_Theta;
    
    const frc::Translation2d SauronBlue = frc::Translation2d(BLUE_LINE_COORD, 4.034663_m);
    const frc::Translation2d LeftPassBlue = frc::Translation2d(1_m, 1.5_m);
    const frc::Translation2d RightPassBlue = frc::Translation2d(1_m, 7_m);
    
    const frc::Translation2d SauronRed = frc::Translation2d(11.915394_m, 4.034663_m);
    const frc::Translation2d LeftPassRed = frc::Translation2d(15.5_m, 1.5_m);
    const frc::Translation2d RightPassRed = frc::Translation2d(15.5_m, 7_m);
    
    std::function<frc::Pose2d()> getRobotBodyPose;
    std::function<frc::Pose2d()> _getBotPose;
    
    std::function<void(frc::Pose2d, units::time::second_t)> setRobotBodyVisionMeasurement;
    void Periodic () override;
    auto Track()  -> void;
    auto Stop() -> void;
    // auto CalculateTheta(frc::Translation2d TargetPose) -> void;
    auto CalculateTheta(frc::Translation2d TargetPose, frc::Pose2d RobotPose) -> units::degree_t;
    auto SetTurretCommand(units::turn_t goal) -> void;
    auto GetPosition() -> units::turn_t;
    auto getTargetTranlation(frc::Pose2d RobotPose) -> frc::Translation2d;
    std::function<units::angle::degree_t()> getIMUAngle;

    



public: //SYSID STUFF

    frc2::sysid::SysIdRoutine characterization{
        frc2::sysid::Config{
            /* This is in radians per second², but SysId only supports "volts per second" */
            units::constants::detail::PI_VAL / 6 * (1_V / 1_s),
            /* This is in radians per second, but SysId only supports "volts" */
            units::constants::detail::PI_VAL * 1_V,
            std::nullopt, // Use default timeout (10 s)
            // Log state with SignalLogger class
            [](frc::sysid::State state)
            {
                SignalLogger::WriteString("SysIdRotation_State", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
            }
        },
        frc2::sysid::Mechanism{
            [this](units::voltage::volt_t output)
            {
                /* output is actually radians per second, but SysId only supports "volts" */
                _Motor->SetVoltage(output);
                /* also log the requested output for SysId */
                SignalLogger::WriteValue("Rotational_Rate", output);
            },
            {},
            this
        }
    };


    frc2::sysid::SysIdRoutine *m_sysIdRoutineToApply = &characterization;
    frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction){return m_sysIdRoutineToApply->Quasistatic(direction);}
    frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction){return m_sysIdRoutineToApply->Dynamic(direction);}

private:
    void setupControllerGains(ctre::phoenix6::configs::TalonFXConfiguration& config)
    {
        config.Slot0.kS = 0.10063; // Add 0.25 V output to overcome static friction
        config.Slot0.kV = 1.1036; // A velocity target of 1 rps results in 0.12 V output
        config.Slot0.kA = 0.13873; // An acceleration of 1 rps/s requires 0.01 V output
        config.Slot0.kP = 200; // A position error of 0.2 rotations results in 12 V output
        config.Slot0.kI = 0.0; // No output for integrated error
        config.Slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
    }

    void setupMagicMotionValues(ctre::phoenix6::configs::TalonFXConfiguration& config)
    {
        // set Motion Magic settings
        auto& motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 64_tps; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 128_tr_per_s_sq; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 640_tr_per_s_cu; // Target jerk of 1600 rps/s/s (0.1 seconds)
    }


    void setupLimitSwitches(ctre::phoenix6::configs::TalonFXConfiguration& config)
    {
        // set Motion Magic settings
        auto& SoftwareLimitSwitchConfig = config.SoftwareLimitSwitch;
        SoftwareLimitSwitchConfig.ForwardSoftLimitEnable = true;
        SoftwareLimitSwitchConfig.ForwardSoftLimitThreshold = units::turn_t(0.25);
        SoftwareLimitSwitchConfig.ReverseSoftLimitEnable = true;
        SoftwareLimitSwitchConfig.ReverseSoftLimitThreshold = units::turn_t(-0.25);
    }    
};