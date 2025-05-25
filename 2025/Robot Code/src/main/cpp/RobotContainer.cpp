// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/WaitCommand.h>
#include "subsystems/Drivetrain.h"
#include "commands/SwerveControlCmd.h"
#include "RobotConstants.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include "subsystems/Winch.h"
#include "subsystems/ServoClamp.h"
#include "LimeLightHelpers.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>


RobotContainer::RobotContainer() 
{
  Pigeon_Sys = std::make_unique<ctre::phoenix6::hardware::Pigeon2>(PIGEON_ID);
  Pigeon_Sys->Reset();

  //Not sure where this should go or if it should be in multiple places?
  //The Elevator or the arm
  //The Elevator and the arm??
  //CoralRampBreakBeam = std::make_unique<frc::DigitalInput>(0);

  ClawElevatorCANdi = std::make_unique<CANDigitalInput>(ClawElevatorCANdi_CANid);
  Command_Sys = std::make_unique<Scoring>();
  Drivetrain_Sys = std::make_unique<Drivetrain>(*Pigeon_Sys);
  Claw_Sys = std::make_unique<Claw>(ClawElevatorCANdi.get());
  Elevate_Sys = std::make_unique<Elevator>(ClawElevatorCANdi.get());
  Algae_Sys = std::make_unique<Algae>();
  Winch_Sys = std::make_unique<Winch>();
  Servo_Sys = std::make_unique<WinchServo>();
  CANdle_Sys = std::make_unique<CANdleSystem>();

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 
{
  DriverCommands();
  OperatorCommands();
  AutomaticCommands();
  AutoNamedCommands();
}

void RobotContainer::DriverCommands()
{
  auto Score = Command_Sys->Score(Elevate_Sys.get(), Claw_Sys.get());
  auto winchIn = std::move(Servo_Sys->UnlockWinch()).AndThen(std::move(frc2::WaitCommand(1_s).ToPtr())).AndThen(std::move(Winch_Sys->Out()));
  auto winchOut = std::move(Servo_Sys->LockWinch()).AndThen(std::move(frc2::WaitCommand(1_s).ToPtr())).AndThen(std::move(Winch_Sys->In()));

  ((Driver.isTranslating()||Driver.isRotating())&&!Driver.Controller->A()).WhileTrue(SwerveControlCmd(*Drivetrain_Sys, Driver, *Pigeon_Sys).ToPtr());
  Driver.Controller->RightTrigger(0.5).OnTrue(std::move(Score).AlongWith(std::move(CANdle_Sys->SetRed())));
  Driver.Controller->LeftBumper().WhileTrue(std::move(winchIn).AlongWith(CANdle_Sys->SetRed()));
  Driver.Controller->RightBumper().WhileTrue(std::move(winchOut).AlongWith(CANdle_Sys->SetGreen()));
  Driver.Controller->Y().WhileTrue(std::move(Algae_Sys->Outfeed()).AlongWith(CANdle_Sys->SetGreen()));
  Driver.Controller->X().WhileTrue(std::move(Algae_Sys->Intake()).AlongWith(CANdle_Sys->SetRed()));
  Driver.Controller->LeftTrigger(0.5).WhileTrue(std::move(Algae_Sys->SetToCollect()));
  Driver.Controller->A().OnTrue(std::move(frc2::cmd::RunOnce([this](){Drivetrain_Sys->SetGyroYaw(0_deg);})));
  Driver.Controller->B().WhileTrue(std::move(frc2::cmd::RunOnce([this](){Algae_Sys->SetToScore();})));
}

void RobotContainer::OperatorCommands()
{
  auto Ready = Command_Sys->Ready(Elevate_Sys.get(), Claw_Sys.get());
  auto L2 = Command_Sys->L2(Elevate_Sys.get(), Claw_Sys.get());
  auto L3 = Command_Sys->L3(Elevate_Sys.get(), Claw_Sys.get());
  auto L4 = Command_Sys->L4(Elevate_Sys.get(), Claw_Sys.get());

  Operator.Controller.Y().OnTrue(std::move(L4).AlongWith(CANdle_Sys->SetBlue()));
  Operator.Controller.B().OnTrue(std::move(L3).AlongWith(CANdle_Sys->SetBlue()));
  Operator.Controller.A().OnTrue(std::move(L2).AlongWith(CANdle_Sys->SetBlue()));
  Operator.Controller.LeftTrigger(0.5).OnTrue(std::move(Ready).AlongWith(std::move(CANdle_Sys->SetYellow())).AndThen(std::move(CANdle_Sys->SetGreen())));
  Operator.Controller.LeftBumper().OnTrue(std::move(Elevate_Sys->SetToLevel(Level::L2_Algae)).AlongWith(std::move(CANdle_Sys->SetCyan())));
  Operator.Controller.RightBumper().OnTrue(std::move(Elevate_Sys->SetToLevel(Level::L3_Algae)).AlongWith(std::move(CANdle_Sys->SetCyan())));
  Operator.Controller.RightTrigger(0.5).OnTrue(std::move(Claw_Sys->SetToCollect()));
  Operator.Controller.POVUp().WhileTrue(std::move(Elevate_Sys->BumpUp()));
  Operator.Controller.POVDown().WhileTrue(std::move(Elevate_Sys->BumpDown()));

  //Operator.Controller.X().OnTrue(std::move(CANdle_Sys->SetFire()));
}

void RobotContainer::AutomaticCommands()
{
  auto Collect = Command_Sys->Collect(Elevate_Sys.get(), Claw_Sys.get());
  (Elevate_Sys->CoralReady()).OnTrue(std::move(Driver.setRumble(0.5)).AlongWith(Operator.setRumble(0.5)));
  (Elevate_Sys->CoralReady()).OnFalse(std::move(Driver.setRumble(0.0).AlongWith(Operator.setRumble(0.0))));
  (Elevate_Sys->L4Score()).OnTrue(std::move(Driver.setRumble(0.5)).AlongWith(std::move(CANdle_Sys->SetRed())));
  (Elevate_Sys->L4Score()).OnFalse(std::move(Driver.setRumble(0.0)));
  (Elevate_Sys->CoralReady()).OnTrue(std::move(frc2::WaitCommand((units::second_t) .7)).AndThen(std::move(Collect).AlongWith(CANdle_Sys->SetRed())));
}

void RobotContainer::AutoNamedCommands()
{
  using namespace pathplanner;
  NamedCommands::registerCommand("StopDrive", frc2::cmd::RunOnce([this]{Drivetrain_Sys->StopAllMotors_();}));
  NamedCommands::registerCommand("L2", std::move(Command_Sys->L2(Elevate_Sys.get(), Claw_Sys.get())));
  NamedCommands::registerCommand("L3", std::move(Command_Sys->L3(Elevate_Sys.get(), Claw_Sys.get())));
  NamedCommands::registerCommand("L4", std::move(Command_Sys->AutoL4(Elevate_Sys.get(), Claw_Sys.get())));
  NamedCommands::registerCommand("Score", std::move(Command_Sys->Score(Elevate_Sys.get(), Claw_Sys.get())));
  NamedCommands::registerCommand("Ready", std::move(Command_Sys->Ready(Elevate_Sys.get(), Claw_Sys.get())));
  NamedCommands::registerCommand("Collect", std::move(frc2::cmd::WaitUntil([&](){return Elevate_Sys->IsCoralInIntake();}).AndThen(std::move(frc2::WaitCommand((units::second_t) .7)).AndThen(std::move(Command_Sys->AutoCollect(Elevate_Sys.get(), Claw_Sys.get()))))));
  
}



void RobotContainer::ScanCANdi()
{
  ClawElevatorCANdi->ScanInputs();
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand(std::string pathPlannerAuto) 
{
  using namespace pathplanner;
  return PathPlannerAuto(pathPlannerAuto).AndThen([this]{Drivetrain_Sys->StopAllMotors_();}); 
}

void RobotContainer::autoUpdate() 
{
  Drivetrain_Sys->UpdateOdoWithCamera();
}

void RobotContainer::ResetPigeonToCamera()
{
  Pigeon_Sys->SetYaw(units::degree_t{LimelightHelpers::getBotpose()[5]});
}

void RobotContainer::ResetPose()
{
  Drivetrain_Sys->ResetOdoWithCamera();
}
