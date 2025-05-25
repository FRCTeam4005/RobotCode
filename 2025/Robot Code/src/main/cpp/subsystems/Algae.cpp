#include "subsystems/Algae.h"
#include "frc2/command/FunctionalCommand.h"
#include "frc/smartdashboard//SmartDashboard.h"
#include <iostream>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>

using namespace AlgaeConstants;

//magnet active is 0 when the magnet is close to the hall effect sensor
#define ALGAE_ARM_IN_UP_STATE false
//magnet idle is 1 (5V) when the magnet is not close to the hall effect sensor
#define ALGAE_ARM_NOT_IN_UP_STATE true

Algae::Algae() 
{
    Algae_Motor_ = std::make_unique<rev::spark::SparkMax>(Algae_Arm, rev::spark::SparkMax::MotorType::kBrushless);
    Intake_Motor_ = std::make_unique<rev::spark::SparkFlex>(Algae_Intake, rev::spark::SparkMax::MotorType::kBrushless);
    
    // Algae_Motor_->SetInverted(true); //May change if necessary
    // Intake_Motor_->SetInverted(true);

    rev::spark::SparkBaseConfig Algae_Motor_Config_;
    rev::spark::SparkBaseConfig Intake_Motor_Config_;

    Algae_PID_ = std::make_unique<rev::spark::SparkClosedLoopController>(Algae_Motor_->GetClosedLoopController());
    Intake_PID_ = std::make_unique<rev::spark::SparkClosedLoopController>(Intake_Motor_->GetClosedLoopController());

    Algae_Encoder_ = std::make_unique<rev::spark::SparkRelativeEncoder>(Algae_Motor_->GetEncoder());
    Intake_Encoder_ = std::make_unique<rev::spark::SparkRelativeEncoder>(Intake_Motor_->GetEncoder());

    Algae_Motor_Config_.SmartCurrentLimit(40);
    Algae_Motor_Config_.closedLoop.Pid(.01, 0, 0, rev::spark::ClosedLoopSlot::kSlot0);
    Algae_Motor_Config_.Inverted(true);
    Algae_Motor_Config_.encoder.VelocityConversionFactor(1);
    Algae_Motor_Config_.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    Algae_Motor_Config_.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder);

    Intake_Motor_Config_.SmartCurrentLimit(40);
    Intake_Motor_Config_.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    Intake_Motor_Config_.Inverted(true);

    Algae_Motor_->Configure(Algae_Motor_Config_, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    Intake_Motor_->Configure(Intake_Motor_Config_, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    AlgaeArm_UpSensor = std::make_unique<frc::DigitalInput>(1);

    SetName("Algae");
}

bool Algae::IsAlgaeArmInUpState()
{
  return (AlgaeArm_UpSensor->Get() == ALGAE_ARM_IN_UP_STATE);
}

frc2::CommandPtr Algae::SetToScore() {
    return frc2::FunctionalCommand(
    [this] {},
    [this] {Algae_Motor_->Set(.1);},
    [this] (bool interrupted) {Algae_Motor_->Set(0);},
    [this] {return false;},
    {}
  ).ToPtr();
}
frc2::CommandPtr Algae::SetToCollect() {
    return frc2::FunctionalCommand(
    [this] {},
    [this] {Algae_Motor_->Set(-0.1);},
    [this] (bool interrupted) {Algae_Motor_->Set(0);},
    [this] {return false;},
    {}
  ).ToPtr();
}

double Algae::GetOutput() {
    return Intake_Motor_->GetOutputCurrent();
}

frc2::CommandPtr Algae::SetRPM(double RPM) {
    return frc2::FunctionalCommand(
    [this] {},
    [RPM, this] {Intake_Motor_->Set(RPM);},
    [this] (bool interrupted) {Intake_Motor_->Set(0);},
    [this] {return false;},
    {}
  ).ToPtr();
}

frc2::CommandPtr Algae::Intake() {
    return SetRPM(1.0);
}

frc2::CommandPtr Algae::Outfeed() {
    return SetRPM(-1.0);
}








