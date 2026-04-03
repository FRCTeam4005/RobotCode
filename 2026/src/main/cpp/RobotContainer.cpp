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
    Turret_Sys = std::make_unique<Turret>([this](){return Localization_Sys->getPose();}, [this](){return drivetrain.GetPigeon2().GetYaw().GetValue();}, [this](){return Localization_Sys->isValid();});
    ShooterHood_Sys = std::make_unique<ShooterHood>();
    ShooterKicker_Sys = std::make_unique<ShooterKicker>();
    ShooterWheels_Sys = std::make_unique<ShooterWheels>([this](){return Localization_Sys->getPose();});
    IntakeConveyor_Sys = std::make_unique<IntakeConveyor>();
    IntakeFrontRoller_Sys = std::make_unique<IntakeFrontRoller>();
    Localization_Sys = std::make_unique<Localization>(
        "limelight-bodycam", 
        drivetrain.GetPigeon2(),
        [this](){return drivetrain.GetState().Pose;}, // just dp this on init
        [this](frc::Pose2d pose){drivetrain.ResetPose(pose); drivetrain.GetPigeon2().SetYaw(pose.Rotation().Degrees());}, 
        [this](frc::Pose2d pose, units::time::second_t Timestamp){drivetrain.AddVisionMeasurement(pose,Timestamp);});

    pnH.EnableCompressorAnalog( MinimumOnPressure, MamimumOffPressure);

    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("New Auto");
    frc::SmartDashboard::PutData("Auto Modes", &autoChooser);
    ConfigureBindings();
    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

void RobotContainer::ConfigureBindings()
{
    // // i am doing this like this because it tell me what the button does (generically) and what the button is 
    Drivetrain(Driver);
    Driver.RightBumper().OnTrue(drivetrain.RunOnce([this] {drivetrain.SeedFieldCentric();}));
    TurretTracking(Operator.Y());
    ShootBall(Operator.B());
    IntakeBall(Operator.X());
    ReverseConveyor(Operator.RightTrigger());
    //Testing(Driver.RightTrigger());


    // Drivetrain(Grammer);
    // ShootBall(Grammer.RightTrigger());
    // IntakeBall(Grammer.LeftTrigger());
    // ReverseConveyor(Grammer.B());
    AutoNamedCommands();



    
    // frc2::RobotModeTriggers::Disabled()
    //     .WhileTrue(loca);


    // Driver.LeftBumper().OnTrue(frc2::cmd::RunOnce(SignalLogger::Start));
    // Driver.RightBumper().OnTrue(frc2::cmd::RunOnce(SignalLogger::Stop));
    // Driver.Y().WhileTrue(Turret_Sys->SysIdQuasistatic(frc2::sysid::Direction::kForward));
    // Driver.A().WhileTrue(Turret_Sys->SysIdQuasistatic(frc2::sysid::Direction::kReverse));
    // Driver.B().WhileTrue(Turret_Sys->SysIdDynamic(frc2::sysid::Direction::kForward));
    // Driver.X().WhileTrue(Turret_Sys->SysIdDynamic(frc2::sysid::Direction::kReverse));
}







void RobotContainer::Drivetrain(const frc2::CommandXboxController& Controller)
{
    drivetrain.SetDefaultCommand(// Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this, &Controller]() -> auto&& { 
            return drive
                .WithVelocityX(-Controller.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-Controller.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(-Controller.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
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
        .ToggleOnTrue(IntakeFrontRoller_Sys->Out());
}

void RobotContainer::ShootBall(frc2::Trigger trigger)
{
    trigger
        .OnTrue(
            frc2::cmd::Sequence
            (
                ShooterWheels_Sys->Spin(),
                ShooterKicker_Sys->Feed(),
                IntakeConveyor_Sys->In()
            ))
        .OnFalse(
            frc2::cmd::Parallel
            (
                ShooterWheels_Sys->Stop(),
                ShooterKicker_Sys->Stop(),
                IntakeConveyor_Sys->Stop()
            ));
}

void RobotContainer::ReverseConveyor( frc2::Trigger trigger)
{
    trigger
        .WhileTrue(IntakeConveyor_Sys->Out().AlongWith(IntakeFrontRoller_Sys->Unstick()));
}

void RobotContainer::AutoNamedCommands()
{
    using namespace pathplanner;
    auto shootBall = ShooterWheels_Sys->AutoSpin()
                    .AndThen(ShooterKicker_Sys->Feed())
                    .AndThen(IntakeConveyor_Sys->In());

    auto stopShoot = ShooterWheels_Sys->Stop()
                    .AlongWith(ShooterKicker_Sys->Stop())
                    //.AlongWith(IntakeFrontRoller_Sys->In())
                    .AlongWith(IntakeConveyor_Sys->Stop());
    
    auto intakeBall = IntakeFrontRoller_Sys->AutoOut();
    //auto noIntake = IntakeFrontRoller_Sys->In();

    
    NamedCommands::registerCommand("Shoot Ball", std::move(shootBall));
    NamedCommands::registerCommand("Stop Shoot", std::move(stopShoot));
    NamedCommands::registerCommand("Intake Ball", std::move(intakeBall));
    //NamedCommands::registerCommand("Intake Up", std::move(noIntake));

}

// void RobotContainer::Testing(frc2::Trigger trigger)
// {
//     trigger
//         .ToggleOnTrue(IntakeConveyor_Sys->In());
// }

