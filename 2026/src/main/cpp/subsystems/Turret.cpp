#include "subsystems/Turret.h"

Turret::Turret()
{
    TurretMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kTurretMotorID);
    ctre::phoenix6::configs::TalonFXConfiguration turret_cfg{};
    ctre::phoenix6::configs::MotionMagicConfigs &mm = turret_cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 80_tps; // 5 (mechanism) rotations per second cruise
    mm.MotionMagicAcceleration = 160_tr_per_s_sq; // Take approximately 0.5 seconds to reach max vel
    mm.MotionMagicJerk = 1600_tr_per_s_cu;// Take approximately 0.1 seconds to reach max accel 

    turret_cfg.ClosedLoopGeneral.ContinuousWrap = false;

    turret_cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    turret_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    turret_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = units::angle::turn_t(10);
    turret_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    turret_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = units::angle::turn_t(-135);

    turret_cfg.CurrentLimits.StatorCurrentLimit = units::current::ampere_t(80);
    turret_cfg.CurrentLimits.StatorCurrentLimitEnable = false;

    ctre::phoenix6::configs::Slot0Configs &slot0_ = turret_cfg.Slot0;
    slot0_.kS = 0; // Add 0.25 V output to overcome static friction
    slot0_.kV = 1; // A velocity target of 1 rps results in 0.12 V output
    slot0_.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0_.kP = 4; // A position error of 0.2 rotations results in 12 V output
    slot0_.kI = 0; // No output for integrated error
    slot0_.kD = 0; // A velocity error of 1 rps results in 0.5 V output

    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = TurretMotor->GetConfigurator().Apply(turret_cfg);

    TurretMotor->SetPosition(units::turn_t(0));

    SetName("Turret");
}

void Turret::SetTurretCommand(units::turn_t goal) {
    TurretMotor->SetControl(elevate_mmReq.WithPosition(goal).WithSlot(0));
}

units::turn_t Turret::GetPosition() {
    return TurretMotor->GetRotorPosition().GetValue();
}

frc2::CommandPtr Turret::Move(units::turn_t goal) {
    return frc2::FunctionalCommand(
    [this] {},
    [goal, this] {SetTurretCommand(goal);},
    [this] (bool interrupted) {},
    [goal, this] {return (std::fabs(goal.value() - GetPosition().value()) < 1);},
    {this}
  ).ToPtr();
}

