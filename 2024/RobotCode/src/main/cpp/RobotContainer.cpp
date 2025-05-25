#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/Drivetrain.h"
#include "commands/SwerveControlCmd.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <iostream>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/RunCommand.h>
#include "commands/Extension.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/WaitCommand.h>
#include "subsystems/LimeLightHelpers/LimelightHelpers.h"



using namespace pathplanner;
using namespace SwerveDriveConstants;
using namespace ArmConstants;
using namespace frc2;

// * here are some examples for swervedrivecommand 
// * https://github.com/SeanSun6814/FRC0ToAutonomous/blob/master/%236%20Swerve%20Drive%20Auto/src/main/java/frc/robot/RobotContainer.java // this is the same guy that made "zero to autoSwerve"
// * https://github.com/wpilibsuite/allwpilib/tree/main/wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand

RobotContainer::RobotContainer() 
{
  Pigeon_Sys = std::make_unique<ctre::phoenix6::hardware::Pigeon2>(CANConstants::PIGEON_ID);
  Pigeon_Sys->Reset();

  m_LimeLightData = std::make_unique<LimeLightData>(frc::DriverStation::GetAlliance());
  m_LimeLight = std::make_unique<LimeLight>(m_LimeLightData.get());
  
  Drivetrain_Sys = std::make_unique<Drivetrain>(*Pigeon_Sys, m_LimeLightData.get());
  Arm_Sys = std::make_unique<Arm>(); // maybe this

  Pneumatics_Sys = std::make_unique<Pneumatics>();
  Extender_Sys = std::make_unique<Extender>(Pneumatics_Sys.get());
  Climber_Sys = std::make_unique<Climber>(Pneumatics_Sys.get());
  Extensions_Cmds = std::make_unique<Extension>();
  Intake_Sys = std::make_unique<Intake>(&Driver, &Operator);
  PDH = std::make_unique<frc::PowerDistribution>(CANConstants::PDH_ID, frc::PowerDistribution::ModuleType::kRev);
  Shooter_Sys = std::make_unique<Shooter>();


  ConfigureBindings();
}


void RobotContainer::ConfigureBindings() 
{

  NamedCommands::registerCommand("StopDrive", frc2::cmd::RunOnce([this]{Drivetrain_Sys->StopAllMotors_();}));
  NamedCommands::registerCommand("SpinUpShooter", std::move(Shooter_Sys->SetShootSpeed(60_tps, 60_tps))); //Change if not shooting enough
  NamedCommands::registerCommand("idleShooter", std::move(Shooter_Sys->SetShootSpeed(40_tps, 40_tps))); //Change if not shooting enough
  NamedCommands::registerCommand("StopShooter", std::move(Shooter_Sys->SetShootSpeed(0_tps, 0_tps))); 
  NamedCommands::registerCommand("FeedNoteToShooter", std::move(
    Arm_Sys->SetSlightlyAboveFloor()
    .AndThen(frc2::cmd::Wait(0.5_s))
    .AndThen(Intake_Sys->FeedNoteToShooter()
    .AndThen(frc2::cmd::Wait(0.5_s))
    .AndThen(Arm_Sys->SetBelowBumperPosition()))));
  NamedCommands::registerCommand("FeedNoteToShooterSide", std::move(Intake_Sys->FeedNoteToShooter()));
  NamedCommands::registerCommand("GetNoteOffFloor", std::move(Intake_Sys->GetNoteOffFloor()));
  NamedCommands::registerCommand("ExtensionOutCommand", std::move(Extensions_Cmds->extensionOut(Extender_Sys.get(), Arm_Sys.get(), Intake_Sys.get())));
  NamedCommands::registerCommand("ExtensionInCommand", std::move(Extensions_Cmds->extensionIn(Extender_Sys.get(), Arm_Sys.get(), Intake_Sys.get())));
  
  ((Driver.isTranslating()||Driver.isRotating())&&!Driver.Controller->A()).WhileTrue(SwerveControlCmd(*Drivetrain_Sys, Driver, *Pigeon_Sys).ToPtr());
  
  //Intake In
  auto inCmd = Extensions_Cmds->extensionIn(Extender_Sys.get(),Arm_Sys.get(), Intake_Sys.get());
  Operator.Controller.A().OnTrue(std::move(inCmd)); //In
  //Intake Out
  auto outCmd = Extensions_Cmds->extensionOut(Extender_Sys.get(),Arm_Sys.get(), Intake_Sys.get());
  Operator.Controller.Y().OnTrue(std::move(outCmd)); //Out
  //Shoot
  Operator.Controller.B().WhileTrue(std::move(Shooter_Sys->SetShootSpeed(80_tps, 80_tps)).AndThen(std::move(Intake_Sys->FeedNoteToShooter())));
  Driver.Controller->X().WhileTrue(frc2::cmd::RunOnce([this]{Pigeon_Sys->SetYaw(0_deg);}));

  //Amp Position
  //Slows down the up and down speeds for the climb position
  units::angle::turn_t h = 5_tr;
  Operator.Controller.X().OnTrue(std::move(Arm_Sys->SetDesiredPosition(kClimbPos/4)).AlongWith(Extender_Sys->SetPastBumper())
                                          .AndThen(std::move(WaitCommand((units::second_t) .15)).ToPtr())
                                          .AndThen(Arm_Sys->SetDesiredPosition(kClimbPos/2))
                                          .AndThen(std::move(WaitCommand((units::second_t) .15)).ToPtr())
                                          .AndThen(Arm_Sys->SetDesiredPosition(3*kClimbPos/4))
                                          .AndThen(std::move(WaitCommand((units::second_t) .15)).ToPtr())
                                          .AndThen(Arm_Sys->SetToClimbPosition()));

  Operator.Controller.LeftBumper().OnTrue(std::move(Arm_Sys->SetDesiredPosition(kClimbPos-h)).AlongWith(Extender_Sys->SetIntoFrame())
                           .AndThen(std::move(WaitCommand((units::second_t) .25)).ToPtr())
                           .AndThen(Arm_Sys->SetDesiredPosition(kClimbPos-(2*h)))
                           .AndThen(std::move(WaitCommand((units::second_t) (.25)).ToPtr()))
                           .AndThen(Arm_Sys->SetDesiredPosition(kClimbPos-(3*h)))
                           .AndThen(std::move(WaitCommand((units::second_t) (.25)).ToPtr()))
                           .AndThen(Arm_Sys->SetDesiredPosition(kClimbPos-(4*h)))
                           .AndThen(std::move(WaitCommand((units::second_t) (.25)).ToPtr()))
                           .AndThen(Arm_Sys->SetDesiredPosition(kClimbPos-(5*h)))
                           .AndThen(std::move(WaitCommand((units::second_t) (.25)).ToPtr()))
                           .AndThen(Arm_Sys->SetDesiredPosition(kSetOnSwervePos)));


  Operator.Controller.RightTrigger(0.5).WhileTrue(std::move(Climber_Sys->Ascend()));
  Operator.Controller.LeftTrigger(0.5).WhileTrue(std::move(Climber_Sys->Descend()));
  (!Operator.Controller.LeftTrigger(0.5) && !Operator.Controller.RightTrigger(0.5)).WhileTrue(std::move(Climber_Sys->LockIn()));
  Operator.Controller.RightBumper().WhileTrue(Shooter_Sys->SetShootSpeed(20_tps, 20_tps).AndThen(std::move(Intake_Sys->FeedNoteToShooter())));
  // Driver.Controller->B().OnTrue(PathPlannerAuto("Planner Tuning Auto").AndThen(([this]{StopDrive();}))).OnFalse(frc2::cmd::RunOnce([this]{Drivetrain_Sys->StopAllMotors_();}));q
  // Operator.Controller.A().OnTrue(Arm_Sys.SetArmGoalCommand(25_tr)).
                          // OnFalse(Arm_Sys.SetArmGoalCommand(10_tr).
                          // AndThen(frc2::cmd::Wait(.8_s)).
                          // AndThen(Arm_Sys.SetArmGoalCommand(4_tr)).
                          // AndThen(frc2::cmd::Wait(.8_s)).
                          // AndThen(Arm_Sys.SetArmGoalCommand(0_tr)));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand(std::string& autoname) 
{
  return PathPlannerAuto(autoname).BeforeStarting([this]{ Drivetrain_Sys->ResetOdoWithCamera(); }); 
}

// frc2::CommandPtr RobotContainer::ShootOnceAuto()
// {
//   return Shooter_Sys->SetShootSpeed(80_tps, 80_tps).AndThen(Intake_Sys->FeedNoteToShooter());
// }

frc2::CommandPtr RobotContainer::StopDrive()
{
  return frc2::cmd::RunOnce([this] {Drivetrain_Sys->StopAllMotors_();});
}

void RobotContainer::SetCamToPigeon()
{
  LimelightHelpers::setPipelineIndex(LimeLightConstants::DefaultName,0);
  Pigeon_Sys->SetYaw(units::degree_t{m_LimeLightData->Yaw});
}

void RobotContainer::ResetPose()
{
  Drivetrain_Sys->ResetOdoWithCamera();
}

