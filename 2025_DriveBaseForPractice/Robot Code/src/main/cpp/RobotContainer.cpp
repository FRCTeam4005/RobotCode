// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "subsystems/Drivetrain.h"
#include "commands/SwerveControlCmd.h"
#include "RobotConstants.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>

using namespace pathplanner;

RobotContainer::RobotContainer() 
{
  Pigeon_Sys = std::make_unique<ctre::phoenix6::hardware::Pigeon2>(PIGEON_ID);
  Pigeon_Sys->Reset();

  Drivetrain_Sys = std::make_unique<Drivetrain>(*Pigeon_Sys);

  //PDH = std::make_unique<frc::PowerDistribution>(PDH_ID, frc::PowerDistribution::ModuleType::kRev);


  ConfigureBindings();
}
void RobotContainer::ConfigureBindings() 
{
  
  NamedCommands::registerCommand("StopDrive", frc2::cmd::RunOnce([this]{Drivetrain_Sys->StopAllMotors_();}));

  ((Driver.isTranslating()||Driver.isRotating())&&!Driver.Controller->A()).WhileTrue(SwerveControlCmd(*Drivetrain_Sys, Driver, *Pigeon_Sys).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() 
{
  return PathPlannerAuto("Example Auto").ToPtr(); 

}
