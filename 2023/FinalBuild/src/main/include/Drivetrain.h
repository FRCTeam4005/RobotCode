
#pragma once

#ifndef DRIVE_TRAIN_H
#define DRIVE_TRAIN_H

#include <memory>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include "Pigeon.h"
#include "SwerveModule.h"
#include "RobotConstants.h"
#include <units/angular_velocity.h>
#include <units/angle.h>
#include <units/dimensionless.h>
#include "DriverController.h"

class Drivetrain
{
public:
  Drivetrain(Drivetrain &other) = delete;
  Drivetrain(Drivetrain &&other) = delete;
  void operator=(const Drivetrain &) = delete;
  void operator=(const Drivetrain &&) = delete;

  void teleOpSwerve(DriverController &DC);
  void autoSwerve(units::meters_per_second_t fwd, units::meters_per_second_t str, units::degrees_per_second_t rcw);
  void DriveTrain(DriverController Controller);
  void AlignAllMotors();
  void PrintEncoders();
  void ZeroAllMotors();
  void ParkingBrake();
  void CalibrateTurnEncoder();
  void swerveCalculation(units::meters_per_second_t fwd, units::meters_per_second_t str, units::degrees_per_second_t rcw);
  void fieldCentricSwerveCalculation(units::meters_per_second_t fwd, units::meters_per_second_t str, units::degrees_per_second_t rcw);
  static Drivetrain& getInstance();
  
private:
  Drivetrain();
  void SetDesiredState(frc::SwerveModuleState& desiredState, units::angle::degree_t CurrAngle, SwerveModule& swmodule);
  void SetDesiredTeleOpState(frc::SwerveModuleState& desiredState, units::angle::degree_t CurrAngle, SwerveModule& swmodule);
  void SetAllModuleStates();
  void TeleOpSetAllModuleStates();
  void GetModuleAngles();
  void LowerGearDrive();
  units::degrees_per_second_t RotationPidCalculatione(units::degrees_per_second_t desiredVelocity);

  units::dimensionless::scalar_t LowerGearDivider{2};

  frc::Translation2d m_frontRightLocation{0.305_m, -0.2143_m};
  frc::Translation2d m_frontLeftLocation{0.305_m, 0.2143_m};
  frc::Translation2d m_backRightLocation{-0.305_m, -0.2143_m};
  frc::Translation2d m_backLeftLocation{-0.305_m, 0.2143_m};

  frc::SwerveDriveKinematics<4> m_Kinematics{ m_frontRightLocation, 
                                              m_frontLeftLocation,
                                              m_backRightLocation,
                                              m_backLeftLocation };

  SwerveModule frModule{  FR_DRIVE_ID, 
                          FR_DRIVE_INVERT, 
                          FR_DRIVE_KP, 
                          FR_TURN_ID, 
                          FR_TURN_INVERT,
                          FR_CANCODER};

  SwerveModule flModule{  FL_DRIVE_ID, 
                          FL_DRIVE_INVERT, 
                          FL_DRIVE_KP, 
                          FL_TURN_ID, 
                          FL_TURN_INVERT,
                          FL_CANCODER};


  SwerveModule brModule{  BR_DRIVE_ID, 
                          BR_DRIVE_INVERT, 
                          BR_DRIVE_KP,
                          BR_TURN_ID,
                          BR_TURN_INVERT,
                          BR_CANCODER};


  SwerveModule blModule{  BL_DRIVE_ID, 
                          BL_DRIVE_INVERT, 
                          BL_DRIVE_KP,
                          BL_TURN_ID,
                          BL_TURN_INVERT,
                          BL_CANCODER};


  frc::SwerveModuleState frState;
  frc::SwerveModuleState flState;
  frc::SwerveModuleState brState;
  frc::SwerveModuleState blState;
  
  frc::PIDController Pos_RCWpid{Pos_RCW_P_Gain, Pos_RCW_I_Gain, Pos_RCW_D_Gain};
  //frc::PIDController Vel_RCWpid{Pos_RCW_P_Gain, Pos_RCW_I_Gain, Pos_RCW_D_Gain};
  frc::PIDController Vel_RCWpid{0,0,0,10_ms};
};

#endif