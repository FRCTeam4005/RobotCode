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


    pnH.EnableCompressorAnalog( MinimumOnPressure, MamimumOffPressure);
    
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("New Auto");
    frc::SmartDashboard::PutData("Auto Modes", &autoChooser);
    
    ConfigureBindings();

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

void RobotContainer::ConfigureBindings()
{
    Drivetrain(Driver);

    //ShootBall
    Operator.B()
        .OnTrue(
            frc2::cmd::Sequence
            (
                IntakeFrontRoller_Sys->Out(),
                ShooterWheels_Sys->Spin(),
                IntakeConveyor_Sys->Out()
            ))
        .OnFalse(
            frc2::cmd::Parallel
            (
                ShooterWheels_Sys->Stop(),
                IntakeConveyor_Sys->Stop(),
                ShooterKicker_Sys->Stop()
            ));

    //Intake Ball
    Operator.X()    
        .OnTrue(IntakeFrontRoller_Sys->Out().AlongWith(frc2::cmd::Print(" \n\n\n INTAKE BALL \n\n\n")))
        .OnFalse(IntakeFrontRoller_Sys->In());
    
    // Reverse Conveyor
    Operator.RightTrigger()
        .OnTrue(IntakeConveyor_Sys->Out())
        .OnFalse(IntakeConveyor_Sys->Stop());

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return frc2::cmd::Sequence(
        ShooterWheels_Sys->Spin(),
        ShooterKicker_Sys->Feed(),
        IntakeConveyor_Sys->In(),
        IntakeFrontRoller_Sys->Out(),
        frc2::cmd::Wait(2_s),
        IntakeConveyor_Sys->In(),
        frc2::cmd::Wait(10_s)
    );
}





void RobotContainer::Drivetrain(const frc2::CommandXboxController& Controller)
{
    drivetrain.SetDefaultCommand(// Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this, &Controller]() -> auto&& { 
            return drive
                .WithVelocityX(-Controller.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-Controller.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(Controller.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
    }));

    frc2::RobotModeTriggers::Disabled()
        .WhileTrue(
            drivetrain.ApplyRequest([] {
                return swerve::requests::Idle{};
            }).IgnoringDisable(true)
        );
}