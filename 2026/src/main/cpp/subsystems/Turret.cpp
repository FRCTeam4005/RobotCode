#include "subsystems/Turret.h"

Turret::Turret()
{
    TurretMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kTurretMotorID);
    ctre::phoenix6::configs::TalonFXConfiguration turret_cfg{};
    ctre::phoenix6::configs::MotionMagicConfigs &mm = turret_cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 80_tps; // 5 (mechanism) rotations per second cruise
    mm.MotionMagicAcceleration = 160_tr_per_s_sq; // Take approximately 0.5 seconds to reach max vel
    mm.MotionMagicJerk = 800_tr_per_s_cu;// Take approximately 0.1 seconds to reach max accel 

    turret_cfg.ClosedLoopGeneral.ContinuousWrap = false;
    turret_cfg.Feedback.SensorToMechanismRatio = 10;

    turret_cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    turret_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turret_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = units::turn_t(3);
    turret_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turret_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = units::turn_t(-3);

    turret_cfg.MotorOutput.Inverted = true;

    turret_cfg.CurrentLimits.StatorCurrentLimit = units::current::ampere_t(80);
    turret_cfg.CurrentLimits.StatorCurrentLimitEnable = false;

    ctre::phoenix6::configs::Slot0Configs &slot0_ = turret_cfg.Slot0;
    slot0_.kS = 0; // Add 0.25 V output to overcome static friction
    slot0_.kV = 0.2; // A velocity target of 1 rps results in 0.12 V output
    slot0_.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0_.kP = 32; // A position error of 0.2 rotations results in 12 V output
    slot0_.kI = 0; // No output for integrated error
    slot0_.kD = 0; // A velocity error of 1 rps results in 0.5 V output

    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = TurretMotor->GetConfigurator().Apply(turret_cfg);

    turret_controller = std::make_unique<frc::PIDController> (0.0045, 0.07, 0.00004);

    Pigeon_Sys = std::make_unique<ctre::phoenix6::hardware::Pigeon2>(45);

    TurretMotor->SetPosition(units::turn_t(0));

    // frc::SmartDashboard::PutNumber("Prop", 0.0045);
    // frc::SmartDashboard::PutNumber("FeedForward", 0.07);
    // frc::SmartDashboard::PutNumber("Derivative", 0.00004);

    SetName("Turret");
}

void Turret::SetTurretCommand(units::turn_t goal) {
  if (double(goal) < -0.4) {
    if (double(goal) + 1 > 0.4)
    {
      goal = goal + units::turn_t(0.8);
    }
    else
    {
      goal = goal + units::turn_t(1);
    }
  }
  else if (double(goal) > 0.4) {
    if (double(goal) - 1 < -0.4)
    {
      goal = goal - units::turn_t(0.8);
    }
    else
    {
      goal = goal - units::turn_t(1);
    }
  }
  
  TurretMotor->SetControl(elevate_mmReq.WithPosition(goal).WithSlot(0));
}

units::turn_t Turret::GetPosition() {
    return (units::turn_t(TurretMotor->GetRotorPosition().GetValue())/10);
}

frc2::CommandPtr Turret::Move(units::turn_t goal) {
    return frc2::FunctionalCommand(
    [this] {},
    [goal, this] {SetTurretCommand(position + goal);},
    [this] (bool interrupted) {},
    [goal, this] {return true;},
    {this}
  ).ToPtr();
}

frc2::CommandPtr Turret::ShootDrivers() {
    return frc2::FunctionalCommand(
    [this] {},
    [this] {SetTurretCommand(units::turn_t(angle));},
    [this] (bool interrupted) {},
    [this] {return false;},
    {this}
  ).ToPtr();
}

frc2::CommandPtr Turret::TrackTag() {
    return frc2::FunctionalCommand(
    [this] {},
    [this] {Track(tx);},
    [this] (bool interrupted) {},
    [this] {return false;},
    {this}
  ).ToPtr();
}


frc2::CommandPtr Turret::StopTrackingTag() {
    return frc2::FunctionalCommand(
    [this] {},
    [this] {Stop();},
    [this] (bool interrupted) {},
    [this] {return false;},
    {this}
  ).ToPtr();
}

void Turret::Track(double offset) {
  auto desiredOutput = turret_controller->Calculate(offset, 0);
  //TurretMotor->Set(desiredOutput);
  if (desiredOutput>0) 
  {
    TurretMotor->Set(desiredOutput + feedforward);
  }
  else if (desiredOutput < 0)
  {
    TurretMotor->Set(desiredOutput - feedforward);
  }
  frc::SmartDashboard::PutNumber("motor output",desiredOutput);
}

void Turret::Stop() {
  auto desiredOutput = 0;
  TurretMotor->Set(desiredOutput);

  frc::SmartDashboard::PutNumber("motor output",desiredOutput);
}
