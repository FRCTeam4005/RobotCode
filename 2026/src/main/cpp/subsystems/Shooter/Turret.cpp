#include "subsystems/Shooter/Turret.h"
#include "subsystems/Drivetrain.h"
#include <cmath>
#include <frc/Timer.h>
#include <ctre/phoenix6/Pigeon2.hpp>


Turret::Turret(std::function<frc::Pose2d()> getBotPose, std::function<units::angle::degree_t()> getIMU)
: _getBotPose{getBotPose},
getIMUAngle{getIMU}
{
    _Motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kTurretMotorID);

    // // in init function
    ctre::phoenix6::configs::TalonFXConfiguration turret_cfg{};

    turret_cfg.ClosedLoopGeneral.ContinuousWrap = true;
    turret_cfg.Feedback.SensorToMechanismRatio = 10;

    // // turret_cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    turret_cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    turret_cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turret_cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = units::turn_t(0.25);
    turret_cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turret_cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = units::turn_t(-0.25);

    turret_cfg.MotorOutput.Inverted = true;

    // // turret_cfg.CurrentLimits.StatorCurrentLimit = units::current::ampere_t(40);
    turret_cfg.CurrentLimits.StatorCurrentLimitEnable = false;


    // // set slot 0 gains
    auto& slot0Configs = turret_cfg.Slot0;

    slot0Configs.kS = 0.10063; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 1.1036; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.13873; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 7.5; // A position error of 0.2 rotations results in 12 V output
    slot0Configs.kI = 0.0; // No output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.5 V output

    // set Motion Magic settings
    auto& motionMagicConfigs = turret_cfg.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 64_tps; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 128_tr_per_s_sq; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 640_tr_per_s_cu; // Target jerk of 1600 rps/s/s (0.1 seconds)

    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = _Motor->GetConfigurator().Apply(turret_cfg);
    
    // _Motor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    _Motor->GetConfigurator().Apply(turret_cfg);
    // _Motor->GetConfigurator().Apply(std::noop);

    _Motor->SetPosition(units::turn_t(0));

    SetName("Turret");
}

void Turret::Periodic ()
{
#if 1
    // _Motor->Set(-.2);

  auto HasLeftHitLimitSwitch =  LeftMagSwitch.GetData().magnetDetected;
  auto HasRightHitLimitSwitch =  RightMagSwtich.GetData().magnetDetected;
  auto HasMiddleHitLimitSwitch =  MiddleMagSwitch.GetData().magnetDetected;


  frc::SmartDashboard::PutBoolean("leftmagswitchvals" , HasLeftHitLimitSwitch );
  frc::SmartDashboard::PutBoolean("middlemagswitchvals" , HasMiddleHitLimitSwitch );
  frc::SmartDashboard::PutBoolean("rightmagswitchvals" , HasRightHitLimitSwitch );
  
  auto angle = frc::Rotation2d{getIMUAngle().convert<units::angle::degree>()};

  auto CurrPose = _getBotPose();
  auto TargetCoords = getTargetTranlation(CurrPose);
  // auto TurretAngle = CalculateTheta(TargetCoords, CurrPose) - angle.Degrees();
  auto TurretAngle = CalculateTheta(TargetCoords, CurrPose) - CurrPose.Rotation().Degrees();
  auto withPigeon = CalculateTheta(TargetCoords, CurrPose) - angle.Degrees();


  frc::SmartDashboard::PutNumber("TurretAngle", _Motor->GetMotorVoltage().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("withPigeon", _Motor->GetMotorVoltage().GetValueAsDouble());
  
  // if (abs((TurretAngle - _Motor->GetPosition().GetValue()).convert<units::angle::degree>().value() < 1))
  // {
    //   _Motor->Set(0);
    // }
    // else
    // {
      // frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue ? TurretAngle : TurretAngle + 180_deg;
      
      // if(HasRightHitLimitSwitch)
      // {
      //   _Motor->Set(0);
      //   // _Motor->SetPosition(units::turn_t(0.25));
      // }
      // else if (HasLeftHitLimitSwitch)
      // {
      //   _Motor->Set(0);
      //   // _Motor->SetPosition(units::turn_t(-0.25));
      // }
      // else if (HasMiddleHitLimitSwitch)
      // {
      //   // _Motor->SetPosition(units::turn_t(0));
      // }
      
    // auto deltaAngle = (_Motor->GetPosition().GetValue().convert<units::angle::degree>() + CurrPose.Rotation().Degrees()) - TurretAngle;
    // frc::SmartDashboard::PutNumber("deltaAngle", deltaAngle.value());


  // if (abs(deltaAngle.value()) > 1)
  // {
    this->elevate_mmReq.WithPosition(withPigeon ).WithSlot(0);
    _Motor->SetControl(elevate_mmReq);
  // }
  // else
  // {
  //   _Motor->Set(0);
  // }
#endif
}

// void Turret::updateField(frc::Pose2d robotfieldpose,  frc::Pose2d desiredPose)
// {
//     m_field.SetRobotPose(robotfieldpose);
//   //frc::SmartDashboard::PutData("Current Filed State", &m_field);

//   m_DesiredPoseField.SetRobotPose(desiredPose);
//   //frc::SmartDashboard::PutData("Desired Field State", &m_DesiredPoseField);
// }


frc::Translation2d Turret::getTargetTranlation(frc::Pose2d RobotPose)
{

  // frc::Translation2d DesiredAimCoords;

  // if(frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed)
  // {
  //   if (RobotPose.X() > RED_LINE_COORD)
  //   {
  //     DesiredAimCoords = SauronRed;
  //   }
  //   else
  //   {

  //     if(RobotPose.Y() < MID_FIELD_LINE)
  //     {
  //       DesiredAimCoords = (LeftPassRed);
  //     }
  //     else
  //     {
  //       DesiredAimCoords = (RightPassRed);
  //     }
  //   }
  // }
  // else
  // {
  //   if (RobotPose.X() < BLUE_LINE_COORD)
  //   {
  //     DesiredAimCoords = (SauronBlue);
  //   }
  //   else
  //   {
  //     if(RobotPose.Y() < MID_FIELD_LINE)
  //     {
  //       DesiredAimCoords = (LeftPassBlue);
  //     }
  //     else
  //     {
  //       DesiredAimCoords = (RightPassBlue);
  //     }
  //   }
  // }

  // return DesiredAimCoords;
  return SauronRed;
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





units::degree_t Turret::CalculateTheta(frc::Translation2d TargetPose, frc::Pose2d RobotPose)
{
  frc::Translation2d DeltaPose{TargetPose.X() - RobotPose.X(), TargetPose.Y() - RobotPose.Y()};

  m_Theta = atan((DeltaPose.Y() / DeltaPose.X()).value()) * (180/3.14);

  m_Theta = TargetPose.X() < RobotPose.X() ? m_Theta : m_Theta + 180; 
  distance = std::sqrt(DeltaPose.Y().value() * DeltaPose.Y().value() + DeltaPose.X().value() * DeltaPose.X().value());
  return units::degree_t{m_Theta};
}

void Turret::Stop() {
  auto desiredOutput = 0;
  _Motor->Set(desiredOutput);

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
