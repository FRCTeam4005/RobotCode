
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
#include "RobotConstants.h"

using namespace units::literals;
using namespace frc;

Drivetrain::Drivetrain(ctre::phoenix6::hardware::Pigeon2& pigeon)
: pigeon_{pigeon}
{
  ctre::phoenix6::configs::CANcoderConfiguration  frMagnet{},
  flMagnet{},
  brMagnet{},
  blMagnet{};  

  frMagnet.MagnetSensor.MagnetOffset = FR_cancoder_offset;
  frMagnet.MagnetSensor.SensorDirection = FR_Cancoder_Magnet_sensor_Direction;
  frMagnet.MagnetSensor.AbsoluteSensorDiscontinuityPoint = MagnetSensorRange;
  frModule_ = std::make_unique<SwerveModule>(  FR_DRIVE_ID, 
                                              FR_DRIVE_INVERT, 
                                              FR_DRIVE_KP, 
                                              FR_TURN_ID, 
                                              FR_TURN_INVERT,
                                              FR_CANCODER,
                                              frMagnet );

  flMagnet.MagnetSensor.MagnetOffset = FL_cancoder_offset;
  flMagnet.MagnetSensor.SensorDirection = FL_Cancoder_Magnet_sensor_Direction;
  flMagnet.MagnetSensor.AbsoluteSensorDiscontinuityPoint = MagnetSensorRange;
  flModule_ = std::make_unique<SwerveModule>(  FL_DRIVE_ID, 
                                              FL_DRIVE_INVERT, 
                                              FL_DRIVE_KP, 
                                              FL_TURN_ID, 
                                              FL_TURN_INVERT,
                                              FL_CANCODER,
                                              flMagnet );

  brMagnet.MagnetSensor.MagnetOffset = BR_cancoder_offset;
  brMagnet.MagnetSensor.SensorDirection = BR_Cancoder_Magnet_sensor_Direction;
  brMagnet.MagnetSensor.AbsoluteSensorDiscontinuityPoint = MagnetSensorRange;
  brModule_ = std::make_unique<SwerveModule>(  BR_DRIVE_ID, 
                                              BR_DRIVE_INVERT, 
                                              BR_DRIVE_KP,
                                              BR_TURN_ID,
                                              BR_TURN_INVERT,
                                              BR_CANCODER,
                                              brMagnet );

  blMagnet.MagnetSensor.MagnetOffset = BL_cancoder_offset;
  blMagnet.MagnetSensor.SensorDirection = BL_Cancoder_Magnet_sensor_Direction;
  blMagnet.MagnetSensor.AbsoluteSensorDiscontinuityPoint = MagnetSensorRange;
  blModule_ = std::make_unique<SwerveModule>(  BL_DRIVE_ID, 
                                              BL_DRIVE_INVERT, 
                                              BL_DRIVE_KP,
                                              BL_TURN_ID,
                                              BL_TURN_INVERT,
                                              BL_CANCODER,
                                              blMagnet );
}


void Drivetrain::Periodic()
{

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

void Drivetrain::AutoDrive_( const frc::ChassisSpeeds& speeds)
{
  auto states = SwerveDriveConstants::Kinematics.ToSwerveModuleStates(speeds);
  SetAllModuleStates_(states);
}

void Drivetrain::SetChassisSpeeds( const frc::ChassisSpeeds& speeds, units::second_t period)
{
  SetAllModuleStates_(SwerveDriveConstants::Kinematics.ToSwerveModuleStates(speeds));
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

// frc::Pose2d Drivetrain::GetCameraPose()
// {
//   using namespace units;
//   using namespace frc;
//   auto dat = m_LimeLightData_;
//   return {meter_t{dat->X},meter_t{dat->Y},degree_t{dat->Yaw}};
// }

// void Drivetrain::ResetOdoWithCamera()
// {
//   using namespace units;

//   PoseEstimator->ResetPosition(
//   GetCameraPose().Rotation(),
//   GetAllSwerveModulePositions_(),
//   frc::Pose2d{
//     meter_t{m_LimeLightData_->X},
//     meter_t{m_LimeLightData_->Y},
//     degree_t{m_LimeLightData_->Yaw}});
// }

// void Drivetrain::UpdateOdoWithCamera()
// {
//   PoseEstimator->AddVisionMeasurement(GetCameraPose(),frc::DriverStation::GetMatchTime());
// }

