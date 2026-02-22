#include "subsystems/Turret.h"
#include "subsystems/Drivetrain.h"
#include <cmath>


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
    turret_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    turret_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = units::turn_t(1);
    turret_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    turret_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = units::turn_t(-1);

    turret_cfg.MotorOutput.Inverted = false;

    turret_cfg.CurrentLimits.StatorCurrentLimit = units::current::ampere_t(80);
    turret_cfg.CurrentLimits.StatorCurrentLimitEnable = false;

    ctre::phoenix6::configs::Slot0Configs &slot0_ = turret_cfg.Slot0;

    slot0_.kS = 0; // Add 0.25 V output to overcome static friction
    slot0_.kV = 0.4; // A velocity target of 1 rps results in 0.12 V output
    slot0_.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0_.kP = 32; // A position error of 0.2 rotations results in 12 V output
    slot0_.kI = 0; // No output for integrated error
    slot0_.kD = 0; // A velocity error of 1 rps results in 0.5 V output

    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = TurretMotor->GetConfigurator().Apply(turret_cfg);

    turret_controller = std::make_unique<frc::PIDController> (0.000, 0.00, 0.0000);


    frc::Pose2d TurretPose, BodyPose;

    if(TurretTargetAvaliable() && BodyTargetAvaliable())
    {
      TurretMotor->SetPosition(TurretGetPose().Rotation().Degrees());
    }
    else
    {
      TurretMotor->SetPosition(units::turn_t(0));
    }

    frc::SmartDashboard::PutNumber("Prop", 0.0045);
    frc::SmartDashboard::PutNumber("FeedForward", 0.0);
    frc::SmartDashboard::PutNumber("Derivative", 0.0000);

    feedforward = 0.001;


    SetName("Turret");
}



void Turret::SetTurretCommand(units::turn_t goal) {
  while (double(goal) < -0.5) {
    goal = goal + units::turn_t(1.0);
  }
  while(double(goal) > 0.5) {
    goal = goal - units::turn_t(1.0);
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

frc2::CommandPtr Turret::TrackTag(std::function<frc::Pose2d()> getRobotPose) {
    return frc2::FunctionalCommand(
    [this] {},
    [this,getRobotPose] {Track(getRobotPose);},
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

void Turret::CalibratePose()
{
  if(TurretTargetAvaliable() && BodyTargetAvaliable())
  {
    TurretMotor->SetPosition(TurretGetPose().Rotation().Degrees() - BodyGetPose().Rotation().Degrees());
  }
  else
  {
    TurretMotor->SetPosition(units::turn_t(0));
  }
}


void Turret::Track(std::function<frc::Pose2d()> getRobotPose) 
{

  units::meter_t desiredX = 4.625_m;
  units::meter_t desiredY = 4.030_m;

  auto RobotPose = getRobotPose();
  auto TurretYaw = units::angle::degree_t{LimelightHelpers::getIMUData("limelight-turret").yaw};

  //combined pose that is the robots x and y but the cameras yaw
  auto currentPose = frc::Pose2d{RobotPose.X(),RobotPose.Y(),frc::Rotation2d{TurretYaw}};

  frc::SmartDashboard::PutNumber("Turret IMU YAW", TurretYaw.value());


  m_TurretPose = currentPose;


  auto DeltaY = desiredY - currentPose.Y();
  auto DeltaX = desiredX - currentPose.X();
  frc::SmartDashboard::PutNumber("DeltaX", DeltaX.value());
  frc::SmartDashboard::PutNumber("DeltaY", DeltaY.value());


  auto Theta = -units::angle::radian_t{atan(((desiredY - currentPose.Y()) / (desiredX - currentPose.X())).value())};

  frc::SmartDashboard::PutNumber("Theta", Theta.convert<units::degree>().value());

  //[TODO] do liek if halfway down field invert theta

  auto desiredOutput = turret_controller->Calculate(currentPose.Rotation().Degrees().value(), Theta.convert<units::degree>().value());


  frc::SmartDashboard::PutNumber("Turret COntroll Output", desiredOutput);

  m_DesiredPoseField.SetRobotPose(frc::Pose2d{RobotPose.X(),RobotPose.Y(),frc::Rotation2d{units::angle::degree_t{Theta}}});
  frc::SmartDashboard::PutData("Desired Field State", &m_DesiredPoseField);

  if ((LimelightHelpers::getTV("limelight-turret")) == true)
  {
    if (desiredOutput > 0) 
    {
      TurretMotor->Set(desiredOutput + feedforward);
    }
    else if (desiredOutput < 0)
    {
      TurretMotor->Set(desiredOutput - feedforward);
    }
  }
  else
  {
    TurretMotor->Set(0);
  }

  // if (desiredOutput > 0) 
  // {
  //   TurretMotor->Set(desiredOutput + feedforward);
  // }
  // else if (desiredOutput < 0)
  // {
  //   TurretMotor->Set(desiredOutput - feedforward);
  // }
  
  
  // while (double(position) <= -0.5) {
  //   SetTurretCommand(position + units::turn_t(1.0));
  // }
  // while (double(position) >= 0.5) {
  //   SetTurretCommand(position + units::turn_t(-1.0));
  // }
  //frc::SmartDashboard::PutNumber("motor output",desiredOutput);

}

void Turret::Stop() {
  auto desiredOutput = 0;
  TurretMotor->Set(desiredOutput);

  frc::SmartDashboard::PutNumber("motor output",desiredOutput);
}
