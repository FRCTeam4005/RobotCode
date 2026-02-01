// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "LimelightHelpers.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer()
{
    Turret_Sys = std::make_unique<Turret>();
    Shooter_Sys = std::make_unique<Shooter>();
    Intake_Sys = std::make_unique<Intake>();

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-Driver.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-Driver.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(-Driver.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        })
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    DriverControls();
    OperatorControls();

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

// If this doesn't work, these all need to go back into ConfigureBindings()
void RobotContainer::DriverControls()
{
    Driver.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    Driver.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
       return point.WithModuleDirection(frc::Rotation2d{-Driver.GetLeftY(), -Driver.GetLeftX()});
    }));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    //(Driver.Back() && Driver.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    //(Driver.Back() && Driver.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    //(Driver.Start() && Driver.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    //(Driver.Start() && Driver.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    Driver.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));
    Driver.B().WhileTrue(std::move(Shooter_Sys->SetShootSpeed()));
    Driver.LeftTrigger(0.5).WhileTrue(std::move(Intake_Sys->FuelUp()));
   // Driver.LeftTrigger(0.5).OnTrue(std::move(Intake_Sys->FuelUp()));
}

void RobotContainer::OperatorControls()
{
    //These should just test if the turret works
    Operator.B().OnTrue(std::move(Turret_Sys->TrackTag()));
    Operator.B().OnFalse(std::move(Turret_Sys->StopTrackingTag()));
    Operator.X().OnTrue(std::move(Turret_Sys->Move(units::turn_t(0.1))));
    Operator.A().OnTrue(std::move(Turret_Sys->Move(units::turn_t(-0.1))));
    
    

    //Hoping this will face the turret to the drivers
    //Change the 4096 to however many "ticks" are in one full revolution of the turret
    //Operator.A().OnTrue(std::move(Turret_Sys->ShootDrivers()));
    
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    // Simple drive forward auton
   return frc2::cmd::Sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
       drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(frc::Rotation2d{0_deg}); }),
        // Then slowly drive forward (away from us) for 5 seconds.
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(0.5_mps)
                .WithVelocityY(0_mps)
                .WithRotationalRate(0_tps);
        })
        .WithTimeout(5_s),
        // Finally idle for the rest of auton
        drivetrain.ApplyRequest([] { return swerve::requests::Idle{}; })
    );
}
