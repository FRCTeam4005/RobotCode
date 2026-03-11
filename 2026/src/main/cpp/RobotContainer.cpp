// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Subsystems/Vision/LimelightHelpers.h"
#include <algorithm>

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

RobotContainer::RobotContainer()
{
    BodyCam_Sys = std::make_shared<Vision>("limelight-bodycam");
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
}

void RobotContainer::ConfigureBindings()
{
    DriverControls(Driver);
    OperatorControls(Operator);

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}


// void RobotContainer::SwerveDrive(const frc2::CommandXboxController& Controller)
// {
//     drivetrain.SetDefaultCommand(// Drivetrain will execute this command periodically
//     drivetrain.ApplyRequest([this, &Controller]() -> auto&& { return drive
//             .WithVelocityX(-Controller.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
//             .WithVelocityY(-Controller.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
//             .WithRotationalRate(Controller.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
//     }));

//     frc2::RobotModeTriggers::Disabled()
//     .WhileTrue(
//         drivetrain.ApplyRequest([] {
//             return swerve::requests::Idle{};
//         }).IgnoringDisable(true)
//     );
// }

// void RobotContainer::SwerveDrive(const frc2::CommandXboxController& Controller)
// {
//     Controller.RightTrigger(0.5).WhileTrue(std::move(Turret_Sys->ToggleTracking()));
// }


// If this doesn't work, these all need to go back into ConfigureBindings()
void RobotContainer::DriverControls(const frc2::CommandXboxController& Controller)
{
    drivetrain.SetDefaultCommand(// Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this, &Controller]() -> auto&& { return drive
                .WithVelocityX(-Controller.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-Controller.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(Controller.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        }));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled()
        .WhileTrue(
            drivetrain.ApplyRequest([] {
                return swerve::requests::Idle{};
            }).IgnoringDisable(true)
        );

    //Tracking targets
    Controller.RightTrigger(0.5).WhileTrue(std::move(Turret_Sys->ToggleTracking()));
}

void RobotContainer::OperatorControls(const frc2::CommandXboxController& Controller)
{
    auto IntakeBall = [&](){return IntakeFrontRoller_Sys->Out();};
    auto StopIntaking = [&](){return IntakeFrontRoller_Sys->In();};
    auto ConveyTowardsIntake = [&](){return IntakeConveyor_Sys->Out();};
    auto ConveyTowardsShooter = [&](){return IntakeConveyor_Sys->In();};
    auto ConveyorStop= [&](){return IntakeConveyor_Sys->Stop();};
    auto FeedShooter = [&](){return IntakeConveyor_Sys->Out();};
    auto SpinUpShooter = [&](){return ShooterWheels_Sys->Spin();};
    auto StopShooter = [&](){return ShooterWheels_Sys->Stop();};
    auto StopKicker = [&](){return  ShooterKicker_Sys->Stop();};

    auto ShootBall = [&](){return IntakeBall().AndThen(SpinUpShooter()).AndThen(FeedShooter());};
    auto StopShooting = [&](){return StopShooter().AlongWith(ConveyorStop()).AlongWith(StopKicker());};



    Controller.X()
        .OnTrue(IntakeBall())
        .OnFalse(StopIntaking());
    Controller.B()
        .OnTrue(ShootBall())
        .OnFalse(StopShooting());
    Controller.RightTrigger(0.5)
        .OnTrue(ConveyTowardsIntake())
        .OnFalse(ConveyorStop());
    // Operator.Y().OnTrue(std::move(ShooterHood_Sys->Toggle()));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    auto Commands = frc2::cmd::Sequence(
        ShooterWheels_Sys->Spin().AndThen(std::move(ShooterKicker_Sys->Feed())).AndThen(IntakeConveyor_Sys->In()),
        IntakeFrontRoller_Sys->Out(),
        frc2::cmd::Wait(2_s),
        IntakeConveyor_Sys->In(),
        frc2::cmd::Wait(10_s)
    );

    return Commands;
}