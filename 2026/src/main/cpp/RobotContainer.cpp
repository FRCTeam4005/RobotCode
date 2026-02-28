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

    DriverControls();
    OperatorControls();

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

// If this doesn't work, these all need to go back into ConfigureBindings()
void RobotContainer::DriverControls()
{

    Driver.RightTrigger(0.5).WhileTrue(std::move(Turret_Sys->TrackTag([this](){return drivetrain.GetState().Pose;})));

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
    Driver.LeftTrigger(0.5).WhileTrue(std::move(Intake_Sys->FuelUp()));
    // Driver.RightTrigger(0.5).WhileTrue(std::move(Turret_Sys->ShootDrivers()));
}

void RobotContainer::OperatorControls()
{
    //These should just test if the turret works
    //Operator.B().OnTrue(std::move(Turret_Sys->TrackTag()));
    Operator.B().OnFalse(std::move(Turret_Sys->StopTrackingTag()));
    
    
    

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

void RobotContainer::CalibrateSensors()
{
    //the internal IMU just sets the megatag2 yaw to 0 on start so we yoink it from megatag 1 since megatag one does know the yaw but is just not stable most of the time
    auto UnstableYaw = LimelightHelpers::getBotPose2d_wpiBlue("limelight-bodycam").Rotation().Degrees().value();
    LimelightHelpers::SetRobotOrientation("limelight-bodycam",UnstableYaw,0,0,0,0,0);

    //the internal IMU just sets the megatag2 yaw to 0 on start so we yoink it from megatag 1 since megatag one does know the yaw but is just not stable most of the time
    UnstableYaw = LimelightHelpers::getBotPose2d_wpiBlue("limelight-turret").Rotation().Degrees().value();
    LimelightHelpers::SetRobotOrientation("limelight-turret",UnstableYaw,0,0,0,0,0);

    //set the pose of the drive train pose to match with the 
    drivetrain.ResetPose(BodyGetPose());

    Turret_Sys->CalibratePose();
}