// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "LimelightHelpers.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc2/command/WaitCommand.h>

RobotContainer::RobotContainer()
{
    

    Turret_Sys = std::make_unique<Turret>([this](){return drivetrain.GetState().Pose;},[this](frc::Pose2d visionRobotPose, units::time::second_t Timestamp){drivetrain.AddVisionMeasurement(visionRobotPose,Timestamp);});
    Shooter_Sys = std::make_unique<Shooter>(Turret_Sys.get());
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



    //Tracking targets
    Driver.RightTrigger(0.5).WhileTrue(std::move(Turret_Sys->ToggleTracking()));

    //Shooting/Passing
    //Driver.B().OnTrue(Shooter_Sys->SetShootSpeed(54_tps).AndThen(Shooter_Sys->FeedShooter()));

    //Driver.A().OnTrue(Intake_Sys->FuelUp());


    // Driver.LeftTrigger(0.5).WhileTrue(std::move(Intake_Sys->FuelUp()));
    // Driver.RightTrigger(0.5).WhileTrue(std::move(Intake_Sys->FuelOut()));
    // Driver.RightTrigger(0.5).WhileTrue(std::move(Turret_Sys->ShootDrivers()));
}

void RobotContainer::OperatorControls()
{
    //These should just test if the turret works
    Operator.X().OnTrue(std::move(Intake_Sys->IntakeToggle()));
    //TODO: Ask if this should go to the driver
    Operator.Y().OnTrue(Shooter_Sys->ShooterToggle());
    Operator.B().OnTrue(Shooter_Sys->SetShootSpeed(54_tps).AndThen(Shooter_Sys->FeedShooter()));
    Operator.LeftTrigger(0.5).WhileTrue(std::move(Intake_Sys->FuelUp()));
    Operator.RightTrigger(0.5).WhileTrue(std::move(Intake_Sys->FuelOut()));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return Intake_Sys->IntakeOut().AndThen(
            std::move(frc2::WaitCommand(1_s).ToPtr())).AndThen(
            std::move(Shooter_Sys->SetShootSpeed(54_tps))).AndThen(
            std::move(Shooter_Sys->FeedShooter())).AndThen(
            std::move(frc2::WaitCommand(1_s).ToPtr())).AndThen(
            std::move(Intake_Sys->FuelUp())).AndThen(
            std::move(frc2::WaitCommand(4_s).ToPtr())
           );
}

void RobotContainer::CalibrateSensors()
{
    double UnstableYaw;

    //the internal IMU just sets the megatag2 yaw to 0 on start so we yoink it from megatag 1 since megatag one does know the yaw but is just not stable most of the time
    UnstableYaw = LimelightHelpers::getBotPose2d_wpiBlue("limelight-bodycam").Rotation().Degrees().value();
    LimelightHelpers::SetRobotOrientation("limelight-bodycam",UnstableYaw,0,0,0,0,0);

    //the internal IMU just sets the megatag2 yaw to 0 on start so we yoink it from megatag 1 since megatag one does know the yaw but is just not stable most of the time
    // UnstableYaw = LimelightHelpers::getBotPose2d_wpiBlue("limelight-turret").Rotation().Degrees().value();
    // LimelightHelpers::SetRobotOrientation("limelight-turret",UnstableYaw,0,0,0,0,0);

    //set the pose of the drive train pose to match with the 
    drivetrain.ResetPose(BodyGetPose());

    Turret_Sys->CalibratePose();
}