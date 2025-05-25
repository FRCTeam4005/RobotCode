#include "subsystems/Claw.h"
#include "frc2/command/FunctionalCommand.h"
#include "frc/smartdashboard//SmartDashboard.h"
#include <iostream>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <cmath>


using namespace ClawConstants;

Claw::Claw(CANDigitalInput *clawBreakBeam) 
: ClawBreakBeam(clawBreakBeam)
{
  ClawMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kClawMotorID);

  ctre::phoenix6::configs::TalonFXConfiguration cfg{};
  ctre::phoenix6::configs::MotionMagicConfigs &mm = cfg.MotionMagic;
  mm.MotionMagicCruiseVelocity = 240_tps; // 5 (mechanism) rotations per second cruise
  mm.MotionMagicAcceleration = 80_tr_per_s_sq; // Take approximately 0.5 seconds to reach max vel
  // Take approximately 0.1 seconds to reach max accel 
  mm.MotionMagicJerk = 400_tr_per_s_cu;

  cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
  cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = units::angle::turn_t(30);
  cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
  cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = units::angle::turn_t(-80);
  cfg.HardwareLimitSwitch.ForwardLimitEnable = true;
  cfg.HardwareLimitSwitch.ReverseLimitEnable = true;

  cfg.CurrentLimits.StatorCurrentLimit = units::current::ampere_t(80);
  cfg.CurrentLimits.StatorCurrentLimitEnable = false;

  ctre::phoenix6::configs::Slot0Configs &slot0 = cfg.Slot0;
  slot0.kS = 0; // Add 0.25 V output to overcome static friction
  slot0.kV = 2; // A velocity target of 1 rps results in 0.12 V output
  slot0.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
  slot0.kP = 8; // A position error of 0.2 rotations results in 12 V output
  slot0.kI = 0; // No output for integrated error
  slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
  

  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = ClawMotor->GetConfigurator().Apply(cfg);
    if (status.IsOK()) {
      break;
    }
  }

  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }

  ClawMotor->SetPosition(units::turn_t(0));
  SetName("Claw");
}

units::turn_t Claw::GetPosition() {
  return ClawMotor->GetRotorPosition().GetValue();
}

frc2::CommandPtr Claw::SetPosition(ClawPosition position)
{
  return frc2::FunctionalCommand(
        [this, position]{
          switch (position) {
            case ClawPosition::COLLECT:
              //SetToCollect();
            break;
            case ClawPosition::SCORE:
              SetToScore();
            break;
            case ClawPosition::PATH:
              SetToPath();
            break;
            default:
            break;
          }
        },
        [this]{},
        [this](bool interrupted){},
        [this]{return true;},
        {this}
    ).ToPtr();
    }

frc2::CommandPtr Claw::SetToCollect() {
  return frc2::FunctionalCommand(
    [this] {},
    [this] {ClawMotor->Set(-.9);},
    [this] (bool interrupted) {
      ClawMotor->Set(0);
      ClawMotor->SetPosition(-54_tr);
      ClawMotor->SetControl(m_mmReq.WithPosition(ClawConstants::kCollect).WithSlot(0));
      },
    [this] {
      return (ClawBreakBeam->IsClawBreakBeamActive());},
    {}
  ).ToPtr();
}

void Claw::SetToScore() {
  ClawMotor->SetControl(m_mmReq.WithPosition(ClawConstants::kScore).WithSlot(0));
}

void Claw::SetToPath() {
  ClawMotor->SetControl(m_mmReq.WithPosition(ClawConstants::kPath).WithSlot(0));
}



