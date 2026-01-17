#ifndef SWERVE_MODULE_H
#define SWERVE_MODULE_H

#include <rev/SparkMax.h>
#include "frc/Encoder.h"
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/controller/PIDController.h>
#include "rev/RelativeEncoder.h"
#include "units/velocity.h"
#include "Constants.h"
#include <memory>
#include <ctre/phoenix6/CANcoder.hpp>
#include <units/angle.h>
#include <units/length.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <frc/geometry/Rotation2d.h>

using namespace SwerveDriveConstants;

class SwerveModule
{
public:
  SwerveModule( uint32_t drive_id, 
                bool drive_invert,
                double kP,
                uint32_t turn_id,
                bool turn_invert,
                uint32_t turn_encoder_id,
                const ctre::phoenix6::configs::CANcoderConfiguration& turn_CanCoder_Config );
  

  /**
   * @brief causes both the turn and drive motor the spin freely
  */
  void stop();


  /**
   * @param new_state 
   * @brief takes new state and applies it to the module
  */
  auto SetDesiredState(const frc::SwerveModuleState& new_state) -> void;
  auto GetPosition() -> frc::SwerveModulePosition;
 
  frc::SwerveModuleState getState();

private:
  std::unique_ptr<rev::spark::SparkMax> Turn_Motor_;
  std::unique_ptr<rev::spark::SparkBaseConfig> Turn_Motor_Config_;
  std::unique_ptr<rev::spark::SparkMax> Drive_Motor_;
  std::unique_ptr<rev::spark::SparkBaseConfig> Drive_Motor_Config_;

  std::unique_ptr<ctre::phoenix6::hardware::CANcoder> Turn_CanCoder_;
  std::unique_ptr<rev::spark::SparkClosedLoopController>    Drive_PID_;
  // we are using frc pid instead of spark max because we cannot connect absolute mag encoders direcly up to the spark max
  // nor can we directly pass the encoder information into the spark max.
  std::unique_ptr<frc::PIDController>           Turn_PID_;

  std::unique_ptr<rev::spark::SparkRelativeEncoder>  Drive_Encoder_;

  units::meters_per_second_t getDriveVelocity();
  units::degree_t getTurningPostion();
  void Set_Turn_Angle(units::degree_t);

  /**
  * @param velocity -14.5 to 14.5 feet per second 
  * @brief sets the drive motors speed
  * 
  * */ 
  void Set_Drive_Motor_Velocity(units::meters_per_second_t velocity);

};

#endif