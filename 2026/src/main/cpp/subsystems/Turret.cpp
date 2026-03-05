#include "subsystems/Turret.h"
#include "subsystems/Drivetrain.h"
#include <cmath>
#include <frc/Timer.h>


Turret::Turret(std::function<frc::Pose2d()> getRobotPose, std::function<void(frc::Pose2d, units::time::second_t)>setVisionMeasurement) : getRobotBodyPose{getRobotPose}, setRobotBodyVisionMeasurement{setVisionMeasurement}
{
    TurretMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kTurretMotorID);
    ctre::phoenix6::configs::TalonFXConfiguration turret_cfg{};
    ctre::phoenix6::configs::MotionMagicConfigs &mm = turret_cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 80_tps; // 5 (mechanism) rotations per second cruise
    mm.MotionMagicAcceleration = 160_tr_per_s_sq; // Take approximately 0.5 seconds to reach max vel
    mm.MotionMagicJerk = 800_tr_per_s_cu;// Take approximately 0.1 seconds to reach max accel 

    turret_cfg.ClosedLoopGeneral.ContinuousWrap = false;
    turret_cfg.Feedback.SensorToMechanismRatio = 10;

    turret_cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    turret_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turret_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = units::turn_t(0.6);
    turret_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turret_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = units::turn_t(0);

    turret_cfg.MotorOutput.Inverted = true;

    turret_cfg.CurrentLimits.StatorCurrentLimit = units::current::ampere_t(30);
    turret_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    ctre::phoenix6::configs::Slot0Configs &slot0_ = turret_cfg.Slot0;

    slot0_.kS = 0; // Add 0.25 V output to overcome static friction
    slot0_.kV = 0.8; // A velocity target of 1 rps results in 0.12 V output
    slot0_.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0_.kP = 200; // A position error of 0.2 rotations results in 12 V output
    slot0_.kI = 0.01; // No output for integrated error
    slot0_.kD = 0; // A velocity error of 1 rps results in 0.5 V output

    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = TurretMotor->GetConfigurator().Apply(turret_cfg);
    TurretMotor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    turret_controller = std::make_unique<frc::PIDController> (0.0002, 0.00, 0.0000);
    turret_controller->EnableContinuousInput(-180,180);

    frc::Pose2d TurretPose, BodyPose;

    TurretMotor->SetPosition(units::turn_t(0.35));

    // frc::SmartDashboard::PutNumber("Prop", 0.0045);
    // frc::SmartDashboard::PutNumber("FeedForward", 0.);
    // frc::SmartDashboard::PutNumber("Derivative", 0.0000);
    //frc::SmartDashboard::PutData(turret_controller.get());

    feedforward = 0.001;

    SetName("Turret");
}

void Turret::Periodic ()
{
  m_RobotPose = getRobotBodyPose();

  auto pose = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight-bodycam");
  if(pose.tagCount > 0)
  {
    setRobotBodyVisionMeasurement(pose.pose,units::millisecond_t{pose.latency});
  }

  m_TurretCameraPose = TurretGetPose();

  if(frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed)
  {
    if (m_RobotPose.X() > RED_LINE_COORD)
    {
      CalculateTheta(SauronRed);
      hoodUp = false;
    }
    else
    {
      if(m_RobotPose.X() < (RED_LINE_COORD - 3_m))
      {
        hoodUp = true;
      }
      else
      {
        hoodUp = false;
      }

      if(m_RobotPose.Y() < MID_FIELD_LINE)
      {
        CalculateTheta(LeftPassRed);
      }
      else
      {
        CalculateTheta(RightPassRed);
      }
    }
  }
  else
  {
    if (m_RobotPose.X() < BLUE_LINE_COORD)
    {
      CalculateTheta(SauronBlue);
      hoodUp = false;
    }
    else
    {
      if (m_RobotPose.X() > (BLUE_LINE_COORD + 3_m))
      {
        hoodUp = true;
      }
      else 
      {
        hoodUp = false;
      }
      if(m_RobotPose.Y() < MID_FIELD_LINE)
      {
        CalculateTheta(LeftPassBlue);
      }
      else
      {
        CalculateTheta(RightPassBlue);
      }
    }
  }

  position = getTurretPosition();
  auto body = m_RobotPose.Rotation().Degrees().value();
  angle_ = ((m_Theta - body + 122.0)/360.0);

  auto RobotFieldPose = m_RobotPose;

  auto TurretYaw = units::angle::degree_t{LimelightHelpers::getIMUData("limelight-turret").yaw};
  m_TurretPose =  frc::Pose2d{m_RobotPose.X(),m_RobotPose.Y(),{TurretYaw}};
  auto desiredRobotPose = frc::Pose2d{m_RobotPose.X(),m_RobotPose.Y(),frc::Rotation2d{units::angle::degree_t{m_Theta}}};

  frc::SmartDashboard::PutNumber("Turret YAW", TurretYaw.value());

  m_field.SetRobotPose(RobotFieldPose);
  //frc::SmartDashboard::PutData("Current Filed State", &m_field);

  m_DesiredPoseField.SetRobotPose(desiredRobotPose);
  //frc::SmartDashboard::PutData("Desired Field State", &m_DesiredPoseField);

  omega = drivetrain.GetPigeon2().GetAngularVelocityZWorld().GetValueAsDouble();
  //frc::SmartDashboard::PutNumber("Rotation Speed", drivetrain.GetPigeon2().GetAngularVelocityZWorld().GetValueAsDouble());

  if(TurretTrack_)
  {
    SetTurretCommand(units::turn_t(angle_));
  }
}


#if 0
frc2::CommandPtr Turret::ShootDrivers() {
    return frc2::FunctionalCommand(
    [this] {},
    [this] {SetTurretCommand(units::turn_t(angle_));},
    [this] (bool interrupted) {},
    [this] {return false;},
    {this}
  ).ToPtr();
}
#endif

void Turret::SetTurretCommand(units::turn_t goal) {
  //Ensures we are considering equivalent angles
  while (double(goal) < -0.4) {
    goal = goal + units::turn_t(1.0);
  }
  while(double(goal) > 0.6) {
    goal = goal - units::turn_t(1.0);
  }

  //If out of range, will center turret
  if (double(goal) > 0.6 || double(goal) < 0.0)
  {
    goal = units::turn_t(0.3);
  }

  //Run to position
  TurretMotor->SetControl(elevate_mmReq.WithPosition(goal).WithSlot(0));
}

units::turn_t Turret::getTurretPosition() {
  return (units::turn_t(TurretMotor->GetRotorPosition().GetValue())/10);
}

frc2::CommandPtr Turret::TrackTag() {
    return frc2::FunctionalCommand(
    [this] {},
    [this] {Track();},
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
  
}


void Turret::CalculateTheta(frc::Translation2d TargetPose)
{
  frc::Translation2d DeltaPose{TargetPose.X() - m_TurretPose.X(), TargetPose.Y() - m_TurretPose.Y()};

  m_Theta = atan((DeltaPose.Y() / DeltaPose.X()).value()) * (180/3.14);

  //frc::SmartDashboard::PutNumber("Delta Pose X", DeltaPose.X().value());
  //frc::SmartDashboard::PutNumber("Delta Pose Y", DeltaPose.Y().value());
  m_Theta = TargetPose.X() < m_TurretPose.X() ? m_Theta + 180 : m_Theta; 
  distance = std::sqrt(DeltaPose.Y().value() * DeltaPose.Y().value() + DeltaPose.X().value() * DeltaPose.X().value());
  //frc::SmartDashboard::PutNumber("Delta Pose Theta", m_Theta);
  frc::SmartDashboard::PutNumber("Distance", distance);
}

double Turret::GetDistance()
{
  return distance;
}

void Turret::Track() 
{
  auto desiredOutput = turret_controller->Calculate(m_TurretPose.Rotation().Degrees().value(), m_Theta);

  if (desiredOutput > 0) 
  {
    TurretMotor->Set(desiredOutput + feedforward);
  }
  else if (desiredOutput < 0)
  {
    TurretMotor->Set(desiredOutput - feedforward);
  }
}

void Turret::Stop() {
  auto desiredOutput = 0;
  TurretMotor->Set(desiredOutput);

  frc::SmartDashboard::PutNumber("motor output",desiredOutput);
}

bool Turret::IsHoodUp()
{
  return hoodUp;
}

frc2::CommandPtr Turret::ToggleTracking()
{
  return this->RunOnce(
    [this] {TurretTrack_ = !TurretTrack_;}
  );
}
