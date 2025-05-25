// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include "rev/CANSparkMax.h"
#include <frc2/command/WaitCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>


Intake::Intake(DriverController* Driver, OperatorController* Operator)
{
  intakeMotor = std::make_unique<rev::CANSparkMax>(CANConstants::kIntakeMotorID,rev::CANSparkLowLevel::MotorType::kBrushless);
  intakeMotor->SetSmartCurrentLimit(40);
  intakeMotor->SetInverted(false);
  Driver= Driver;
  Operator = Operator;
  SetName("Intake");
  // SetDefaultCommand(frc2::cmd::Run([this] {setSpeed(0);}, {this}));
}

void Intake::setSpeed(double speed)
{
  intakeMotor->Set(speed);
}

frc2::Trigger Intake::IsNoteInIntake()
{
  return frc2::Trigger{ [this] {return !NoteSensor.Get();}};
}

frc2::CommandPtr Intake::FeedNoteToShooter()
{     
  std::cout <<"FeedNoteToShooter\n";
  
  printf("FeedNoteToShooter");
  return frc2::FunctionalCommand(
    [this] {},
    [this] {setSpeed(1);},
    [this] (bool interrupted)
    {
      setSpeed(0);
      Operator->setRumble(1.0);
      Driver->setRumble(1.0);
      printf("Rumble set to 1.0 printf");
      frc2::WaitCommand((units::second_t) (4));
      Operator->setRumble(0.0);
      Driver->setRumble(0.0);
    },
    [this] {return !IsNoteInIntake().Get();},
    {this}
  ).ToPtr();
}

frc2::CommandPtr Intake::GetNoteOffFloor()
{
  return frc2::FunctionalCommand(
    [this] {},
    [this] {setSpeed(0.5);},
    [this] (bool interrupted){setSpeed(0);},
    [this] {return IsNoteInIntake().Get();},
    {this}
  ).ToPtr();
}

frc2::CommandPtr Intake::Stop()
{
  return this->RunOnce(
    [this] {setSpeed(0);}
  );
}