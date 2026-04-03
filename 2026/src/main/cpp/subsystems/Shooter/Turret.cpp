#include "subsystems/Shooter/Turret.h"
#include "subsystems/Drivetrain.h"
#include <cmath>
#include <frc/Timer.h>
#include <ctre/phoenix6/Pigeon2.hpp>


Turret::Turret(std::function<frc::Pose2d()> getBotPose, std::function<units::angle::degree_t()> getIMU, std::function<bool()> isPoseValid)
: _getBotPose{getBotPose},
getIMUAngle{getIMU},
_isPoseValid{isPoseValid}
{
 _Motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kTurretMotorID);


    ctre::phoenix6::configs::TalonFXConfiguration turret_cfg{};

    turret_cfg.ClosedLoopGeneral.ContinuousWrap = true;
    turret_cfg.Feedback.SensorToMechanismRatio = 10;
    turret_cfg.CurrentLimits.StatorCurrentLimitEnable = false;
    turret_cfg.MotorOutput.Inverted = true;
    turret_cfg.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    setupLimitSwitches(turret_cfg);

    setupControllerGains(turret_cfg);

    setupMagicMotionValues(turret_cfg);

    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = _Motor->GetConfigurator().Apply(turret_cfg);
    
    _Motor->GetConfigurator().Apply(turret_cfg);

    _Motor->SetPosition(units::turn_t(0));

    SetName("Turret");
}

void Turret::Periodic ()
{
#if 1
  // auto HasLeftHitLimitSwitch =  LeftMagSwitch.GetData().magnetDetected;
  // auto HasRightHitLimitSwitch =  RightMagSwtich.GetData().magnetDetected;
  // auto HasMiddleHitLimitSwitch =  MiddleMagSwitch.GetData().magnetDetected;


  // frc::SmartDashboard::PutBoolean("leftmagswitchvals" , HasLeftHitLimitSwitch );
  // frc::SmartDashboard::PutBoolean("middlemagswitchvals" , HasMiddleHitLimitSwitch );
  // frc::SmartDashboard::PutBoolean("rightmagswitchvals" , HasRightHitLimitSwitch );

  if(!_isPoseValid())
  {
    _Motor->Set(0);
    return;
  }
  
  // auto angle = frc::Rotation2d{units::degree_t{std::fmod(getIMUAngle().value(),180)}};
  
  auto CurrPose = _getBotPose();
  auto TargetCoords = getTargetTranlation(CurrPose);
  auto DesiredRobotAngle = CalculateTheta(TargetCoords,CurrPose);
  auto AngleSetpoint = DesiredRobotAngle - CurrPose.Rotation().Degrees();

  // frc::SmartDashboard::PutNumber("")
  // frc::SmartDashboard::PutNumber("")
  // frc::SmartDashboard::PutNumber("")
  // frc::SmartDashboard::PutNumber("")


  
  if( AngleSetpoint > 90_deg || AngleSetpoint < -90_deg)
  {
    AngleSetpoint = 0_deg;
  }

  // auto AngleSetpoint = CalculateTheta(TargetCoords, CurrPose) - angle.Degrees();
  if(TurretTrack_)
  {
  this->elevate_mmReq.WithPosition(AngleSetpoint).WithSlot(0);
  _Motor->SetControl(elevate_mmReq);
  }
  else{
    this->elevate_mmReq.WithPosition(0_deg).WithSlot(0);
  _Motor->SetControl(elevate_mmReq);
}

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
