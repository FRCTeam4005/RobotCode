// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "LimelightHelpers.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

RobotContainer::RobotContainer()
{
    

    Turret_Sys = std::make_unique<Turret>([this](){return drivetrain.GetState().Pose;},[this](frc::Pose2d visionRobotPose, units::time::second_t Timestamp){drivetrain.AddVisionMeasurement(visionRobotPose,Timestamp);});
    Shooter_Sys = std::make_unique<Shooter>();
    Intake_Sys = std::make_unique<Intake>();

    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("New Auto");
    frc::SmartDashboard::PutData("Auto Modes", &autoChooser);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{

    DriverControls();
    OperatorControls();

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

// If this doesn't work, these all need to go back into ConfigureBindings()
void RobotContainer::DriverControls()
{
    Driver.RightTrigger(0.5).WhileTrue(Turret_Sys->ShootDrivers());
    //Driver.RightTrigger(0.5).OnTrue(std::move(Turret_Sys->TrackTag())).OnFalse(std::move(Turret_Sys->StopTrackingTag()));

    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-Driver.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-Driver.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(Driver.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        })
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );


    
    Driver.B().WhileTrue(std::move(Shooter_Sys->SetShootSpeed(56_tps).AndThen(Shooter_Sys->FeedShooter())));
    // Driver.LeftTrigger(0.5).WhileTrue(std::move(Intake_Sys->FuelUp()));
    // Driver.RightTrigger(0.5).WhileTrue(std::move(Intake_Sys->FuelOut()));
    // Driver.RightTrigger(0.5).WhileTrue(std::move(Turret_Sys->ShootDrivers()));
}

void RobotContainer::OperatorControls()
{
    //These should just test if the turret works
    Operator.B().OnFalse(std::move(Turret_Sys->StopTrackingTag()));
    Operator.X().OnTrue(std::move(Intake_Sys->IntakeToggle()));
    Operator.Y().OnTrue(std::move(Shooter_Sys->ShooterToggle()));
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return autoChooser.GetSelected();
}