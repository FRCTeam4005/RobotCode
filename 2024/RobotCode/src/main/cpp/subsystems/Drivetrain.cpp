
#include "subsystems/SwerveModule.h"
#include "subsystems/Drivetrain.h"
#include <frc/kinematics/SwerveModulePosition.h>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <frc2/command/Commands.h>
#include <frc2/command/TrapezoidProfileSubsystem.h>
#include <frc/DriverStation.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>
#include <frc/geometry/Pose2d.h>
#include "subsystems/LimeLightHelpers/LimelightHelpers.h"

using namespace pathplanner;
using namespace units::literals;

Drivetrain::Drivetrain(ctre::phoenix6::hardware::Pigeon2& pigeon, LimeLightData* limelightData)
: pigeon_{pigeon},
  m_LimeLightData_{limelightData}
{
  ctre::phoenix6::configs::CANcoderConfiguration  frMagnet{},
  flMagnet{},
  brMagnet{},
  blMagnet{};  

  frMagnet.MagnetSensor.MagnetOffset = FR_cancoder_offset;
  frMagnet.MagnetSensor.SensorDirection = FR_Cancoder_Magnet_sensor_Direction;
  frMagnet.MagnetSensor.AbsoluteSensorRange = MagnetSensorRange;
  frModule_ = std::make_unique<SwerveModule>(  FR_DRIVE_ID, 
                                              FR_DRIVE_INVERT, 
                                              FR_DRIVE_KP, 
                                              FR_TURN_ID, 
                                              FR_TURN_INVERT,
                                              FR_CANCODER,
                                              frMagnet );

  flMagnet.MagnetSensor.MagnetOffset = FL_cancoder_offset;
  flMagnet.MagnetSensor.SensorDirection = FL_Cancoder_Magnet_sensor_Direction;
  flMagnet.MagnetSensor.AbsoluteSensorRange = MagnetSensorRange;
  flModule_ = std::make_unique<SwerveModule>(  FL_DRIVE_ID, 
                                              FL_DRIVE_INVERT, 
                                              FL_DRIVE_KP, 
                                              FL_TURN_ID, 
                                              FL_TURN_INVERT,
                                              FL_CANCODER,
                                              flMagnet );

  brMagnet.MagnetSensor.MagnetOffset = BR_cancoder_offset;
  brMagnet.MagnetSensor.SensorDirection = BR_Cancoder_Magnet_sensor_Direction;
  brMagnet.MagnetSensor.AbsoluteSensorRange = MagnetSensorRange;
  brModule_ = std::make_unique<SwerveModule>(  BR_DRIVE_ID, 
                                              BR_DRIVE_INVERT, 
                                              BR_DRIVE_KP,
                                              BR_TURN_ID,
                                              BR_TURN_INVERT,
                                              BR_CANCODER,
                                              brMagnet );

  blMagnet.MagnetSensor.MagnetOffset = BL_cancoder_offset;
  blMagnet.MagnetSensor.SensorDirection = BL_Cancoder_Magnet_sensor_Direction;
  blMagnet.MagnetSensor.AbsoluteSensorRange = MagnetSensorRange;
  blModule_ = std::make_unique<SwerveModule>(  BL_DRIVE_ID, 
                                              BL_DRIVE_INVERT, 
                                              BL_DRIVE_KP,
                                              BL_TURN_ID,
                                              BL_TURN_INVERT,
                                              BL_CANCODER,
                                              blMagnet );
  auto CameraPose = frc::Pose2d{
    units::meter_t{m_LimeLightData_->X},
    units::meter_t{m_LimeLightData_->Y},
    frc::Rotation2d{
    units::degree_t{m_LimeLightData_->Yaw}}};

  PoseEstimator = std::make_unique<frc::SwerveDrivePoseEstimator<4>>( 
    Kinematics,
    CameraPose.Rotation().Degrees(),
    GetAllSwerveModulePositions_(),
    CameraPose,
    OdometryStateDevs,
    CameraStateDevs);

  AutoBuilder::configureHolonomic(
    [this](){ return PoseEstimator->GetEstimatedPosition(); }, // Robot pose supplier
    [this](frc::Pose2d pose){ResetPose(pose);}, // Method to reset odometry_ (will be called if your auto has a starting pose)
    [this](){ return getSpeeds_(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    [this](frc::ChassisSpeeds speeds){ 
      if(!(speeds.vx.value() == 0) || !(speeds.vy.value() == 0) || !(speeds.omega.value() == 0))
      {
        SetChassisSpeeds(speeds);
      }
      else
      {
        StopAllMotors_();
      } 
    }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    SwerveDriveConstants::SweveAutoConfig,
    []() {auto alliance = frc::DriverStation::GetAlliance();
      if (alliance) 
      {
        return alliance.value() == frc::DriverStation::Alliance::kRed;
      }
      return false;
    },
    this);

  telemetry = std::make_unique<frc::Field2d>();
  //ResetPose(GetCameraPose());
  SetName("Drivetrain");

  SetDefaultCommand(frc2::cmd::Run([this] { StopAllMotors_(); }, {this}));
}

void Drivetrain::InitSendable(wpi::SendableBuilder& builder)
{
  builder.SetSmartDashboardType("Arm SubSystem");
  
  builder.AddDoubleProperty("Current X Position",
    [this]{return PoseEstimator->GetEstimatedPosition().X().value();},
    nullptr);

  builder.AddDoubleProperty("Current Y Position",
    [this]{return PoseEstimator->GetEstimatedPosition().Y().value();},
    nullptr);

  builder.AddDoubleProperty("Current Yaw Position",
    [this]{return PoseEstimator->GetEstimatedPosition().Rotation().Degrees().value();},
    nullptr);
}

void Drivetrain::Periodic()
{
  LimelightHelpers::setPipelineIndex(LimeLightConstants::DefaultName, 0);
  PoseEstimator->Update(pigeon_.GetRotation2d(),GetAllSwerveModulePositions_());
  // UpdateOdoWithCamera();
  telemetry->SetRobotPose(PoseEstimator->GetEstimatedPosition());
  frc::SmartDashboard::PutData(telemetry.get());
  frc::SmartDashboard::PutNumber("Camera Yaw",m_LimeLightData_->Yaw);
  frc::SmartDashboard::PutNumber("Pigeon Yaw",GetPose().Rotation().Degrees().value());
}

wpi::array<frc::SwerveModulePosition, 4> Drivetrain::GetAllSwerveModulePositions_()
{
  return  { flModule_->GetPosition(), frModule_->GetPosition(),
            blModule_->GetPosition(), brModule_->GetPosition() };
}

/**
 * @brief takes the new state of all the modules and applies the states to the motors/modules.
*/
void Drivetrain::SetAllModuleStates_(const wpi::array<frc::SwerveModuleState, 4u> &moduleStates)
{
  flModule_->SetDesiredState(moduleStates[0]);
  frModule_->SetDesiredState(moduleStates[1]);
  blModule_->SetDesiredState(moduleStates[2]);
  brModule_->SetDesiredState(moduleStates[3]);
}

/**
 * @brief all the motors (Drive and Turn) spin freely instead of being powered
*/
void Drivetrain::StopAllMotors_()
{
  flModule_->stop();
  frModule_->stop();
  blModule_->stop();
  brModule_->stop();
}

void Drivetrain::SetChassisSpeeds( const frc::ChassisSpeeds& speeds, units::second_t period)
{
  SetAllModuleStates_(SwerveDriveConstants::Kinematics.ToSwerveModuleStates(speeds));
}

frc::Pose2d Drivetrain::GetPose()
{
  return PoseEstimator->GetEstimatedPosition();
}

void Drivetrain::ResetPose(const frc::Pose2d& pose)
{
  PoseEstimator->ResetPosition(  pigeon_.GetRotation2d(), 
                            GetAllSwerveModulePositions_(), 
                            pose );
}

frc::Rotation2d Drivetrain::getRotation2D()
{
  return pigeon_.GetRotation2d();
}

void Drivetrain::SetYaw(units::degree_t angle)
{
  pigeon_.SetYaw(angle);
}

frc::Pose2d Drivetrain::GetCameraPose()
{
  using namespace units;
  using namespace frc;
  auto dat = m_LimeLightData_;
  return {meter_t{dat->X},meter_t{dat->Y},degree_t{dat->Yaw}};
}

void Drivetrain::ResetOdoWithCamera()
{
  using namespace units;

  PoseEstimator->ResetPosition(
  GetCameraPose().Rotation(),
  GetAllSwerveModulePositions_(),
  frc::Pose2d{
    meter_t{m_LimeLightData_->X},
    meter_t{m_LimeLightData_->Y},
    degree_t{m_LimeLightData_->Yaw}});
}

void Drivetrain::UpdateOdoWithCamera()
{
  PoseEstimator->AddVisionMeasurement(GetCameraPose(),frc::DriverStation::GetMatchTime());
}

