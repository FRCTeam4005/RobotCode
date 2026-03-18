// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <algorithm>

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>


RobotContainer::RobotContainer()
{
    Turret_Sys = std::make_unique<Turret>();
    ShooterHood_Sys = std::make_unique<ShooterHood>();
    ShooterKicker_Sys = std::make_unique<ShooterKicker>();
    ShooterWheels_Sys = std::make_unique<ShooterWheels>();
    IntakeConveyor_Sys = std::make_unique<IntakeConveyor>();
    IntakeFrontRoller_Sys = std::make_unique<IntakeFrontRoller>();
    // Localization_Sys = std::make_unique<Localization>(
    //     "limelight-bodycam", 
    //     drivetrain.GetPigeon2(),
    //     [this](){return drivetrain.GetState().Pose;}, // just dp this on init
    //     [this](frc::Pose2d pose){drivetrain.ResetPose(pose);}, 
    //     [this](frc::Pose2d pose, units::time::second_t Timestamp){drivetrain.AddVisionMeasurement(pose,Timestamp);});

    pnH.EnableCompressorAnalog( MinimumOnPressure, MamimumOffPressure);

    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("New Auto");
    frc::SmartDashboard::PutData("Auto Modes", &autoChooser);
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // i am doing this like this because it tell me what the button does (generically) and what the button is 
    Drivetrain(Driver);
    TurretTracking(Driver.RightTrigger());

    IntakeBall(Operator.X());
    ShootBall(Operator.B());
    ReverseConveyor(Operator.RightTrigger());

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

    Driver.LeftBumper().OnTrue(frc2::cmd::RunOnce(SignalLogger::Start));
    Driver.RightBumper().OnTrue(frc2::cmd::RunOnce(SignalLogger::Stop));

    /*
    * Joystick Y = quasistatic forward
    * Joystick A = quasistatic reverse
    * Joystick B = dynamic forward
    * Joystick X = dynamic reverse
    */
    Driver.Y().WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    Driver.A().WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
    Driver.B().WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    Driver.X().WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
}







void RobotContainer::Drivetrain(const frc2::CommandXboxController& Controller)
{
    drivetrain.SetDefaultCommand(// Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this, &Controller]() -> auto&& { 
            return drive
                .WithVelocityX(Controller.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(Controller.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(Controller.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
    }));

    frc2::RobotModeTriggers::Disabled()
        .WhileTrue(
            drivetrain.ApplyRequest([] {
                return swerve::requests::Idle{};
            }).IgnoringDisable(true)
        );
}

void RobotContainer::TurretTracking(frc2::Trigger trigger)
{
    trigger
        .WhileTrue(Turret_Sys->ToggleTracking());
}

void RobotContainer::IntakeBall(frc2::Trigger trigger)
{
    trigger
        .OnTrue(IntakeFrontRoller_Sys->Out().AlongWith(frc2::cmd::Print(" \n\n\n INTAKE BALL \n\n\n")))
        .OnFalse(IntakeFrontRoller_Sys->In());
}

void RobotContainer::ShootBall(frc2::Trigger trigger)
{
    trigger
        .OnTrue(
            frc2::cmd::Sequence
            (
                ShooterWheels_Sys->Spin(),
                IntakeFrontRoller_Sys->Out(),
                IntakeConveyor_Sys->In()
            ))
        .OnFalse(
            frc2::cmd::Parallel
            (
                ShooterWheels_Sys->Stop(),
                IntakeConveyor_Sys->Stop(),
                ShooterKicker_Sys->Stop(),
                IntakeFrontRoller_Sys->In()
            ));
}

void RobotContainer::ReverseConveyor( frc2::Trigger trigger)
{
    trigger
        .OnTrue(IntakeConveyor_Sys->Out())
        .OnFalse(IntakeConveyor_Sys->Stop());
}

