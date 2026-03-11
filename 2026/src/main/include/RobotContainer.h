// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/Drivetrain.h"
#include <frc/PneumaticHub.h>
#include "Telemetry.h"
#include "subsystems/Shooter/Turret.h"
#include "subsystems/Shooter/Hood.h"
#include "subsystems/Shooter/Kicker.h"
#include "subsystems/Shooter/Wheels.h"
#include "subsystems/Conveyor.h"
#include "subsystems/IntakeFrontRoller.h"
#include "subsystems/Vision/Vision.h"

class RobotContainer {

    
public:
    subsystems::Drivetrain drivetrain{TunerConstants::CreateDrivetrain()};
    std::unique_ptr<Turret> Turret_Sys;
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();
    void CalibrateSensors();

private:
    
    std::unique_ptr<ShooterHood> ShooterHood_Sys;
    std::unique_ptr<ShooterKicker> ShooterKicker_Sys;
    std::unique_ptr<ShooterWheels> ShooterWheels_Sys;
    std::unique_ptr<IntakeConveyor> IntakeConveyor_Sys;
    std::unique_ptr<IntakeFrontRoller> IntakeFrontRoller_Sys;
    std::shared_ptr<Vision> BodyCam_Sys;

    frc::PneumaticHub pnH{CANConstants::kPneumaticHub,};
    units::pounds_per_square_inch_t MinimumOnPressure{80};
    units::pounds_per_square_inch_t MamimumOffPressure{95};

    frc2::CommandXboxController Driver{0};
    frc2::CommandXboxController Operator{1};
    
    void ConfigureBindings();
    void OperatorControls(const frc2::CommandXboxController& Controller);
    void DriverControls(const frc2::CommandXboxController& Controller);

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

    double angle;
    frc::SendableChooser<frc2::Command *> autoChooser;

    void Drivetrain(const frc2::CommandXboxController& Controller);
    void TurretTracking(frc2::Trigger trigger);
    void IntakeBall(frc2::Trigger trigger);
    void ShootBall(frc2::Trigger trigger);
    void ReverseConveyor(frc2::Trigger trigger);

};
