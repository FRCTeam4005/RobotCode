#include "subsystems/Arm.h"
#include "frc2/command/FunctionalCommand.h"
#include "frc/smartdashboard//SmartDashboard.h"

using State = frc::TrapezoidProfile<units::turn>::State;
using namespace ArmConstants;

Arm::Arm() : frc2::TrapezoidProfileSubsystem<units::turn>{{200_tps, 640_tr_per_s_sq}, 0_tr}
{
  m_RotLeft = std::make_unique<rev::CANSparkMax>(CANConstants::kArmRotLeftID, rev::CANSparkMax::MotorType::kBrushless);
  m_RotRight = std::make_unique<rev::CANSparkMax>(CANConstants::kArmRotRightID, rev::CANSparkMax::MotorType::kBrushless);
  m_RotLeft->SetSmartCurrentLimit(40);
  m_RotRight->SetSmartCurrentLimit(40);

  m_RotLeft->RestoreFactoryDefaults();
  m_RotRight->RestoreFactoryDefaults();

  m_RotLeft->SetSmartCurrentLimit(40);
  m_RotRight->SetSmartCurrentLimit(40);

  m_RotRight->SetInverted(true);

  m_RotLeft->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  m_RotRight->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

  RotRightPID = std::make_unique<rev::SparkPIDController>(m_RotLeft->GetPIDController());
  RotLeftPID = std::make_unique<rev::SparkPIDController>(m_RotRight->GetPIDController());
  RotRightPID->SetP(kP);
  RotRightPID->SetI(kI);
  RotRightPID->SetD(kD);

  RotLeftPID->SetP(kP);
  RotLeftPID->SetI(kI);
  RotLeftPID->SetD(kD);

  RotLeftEncoder = std::make_unique<rev::SparkRelativeEncoder>(m_RotLeft->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
  RotRightEncoder = std::make_unique<rev::SparkRelativeEncoder>(m_RotRight->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
  SetName("Arm");
  // SetDefaultCommand(frc2::cmd::Run([this] { SetOnSwervePosition(); }, {this}));
}

void Arm::InitSendable(wpi::SendableBuilder& builder)
{
  builder.SetSmartDashboardType("Arm SubSystem");

  builder.AddDoubleProperty("Current Position",
    [this] { return GetPosition().value(); },
    nullptr);

    builder.AddDoubleProperty("Left Current Draw",
    [this] {return m_RotLeft->GetOutputCurrent();},
    nullptr);

    builder.AddDoubleProperty("Right Current Draw",
    [this] {return m_RotRight->GetOutputCurrent();},
    nullptr);
}

void Arm::UseState(State setpoint) 
{
  RotLeftPID->SetReference(setpoint.position.value(), rev::CANSparkBase::ControlType::kPosition);
  RotRightPID->SetReference(setpoint.position.value(), rev::CANSparkBase::ControlType::kPosition);
}

frc2::CommandPtr Arm::SetArmGoalCommand(units::turn_t goal) {
  
  return frc2::cmd::RunOnce([this, goal] { this->SetGoal(goal); this->Enable();}, {this});
}

units::turn_t Arm::GetPosition() {
  return units::turn_t{RotRightEncoder->GetPosition()};
}

frc2::CommandPtr Arm::SetToClimbPosition() {
  return SetArmGoalCommand(ArmConstants::kClimbPos);
}

frc2::CommandPtr Arm::SetAboveBumperPosition() {
  return SetArmGoalCommand(ArmConstants::kAboveBumperPos);
}

frc2::CommandPtr Arm::SetBelowBumperPosition() {
  return SetArmGoalCommand(ArmConstants::kBelowBumperPos);
}

frc2::CommandPtr Arm::SetOnSwervePosition() {
  return SetArmGoalCommand(ArmConstants::kSetOnSwervePos);
}

frc2::CommandPtr Arm::SetSlightlyAboveFloor() {
  return SetArmGoalCommand(ArmConstants::kBelowBumperPos + 3_tr);
}

frc2::CommandPtr Arm::SetDesiredPosition(units::turn_t desiredPosition) {
  return SetArmGoalCommand(desiredPosition);
}

