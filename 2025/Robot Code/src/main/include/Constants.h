// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/voltage.h>
#include <frc/MathUtil.h>
#include <units/constants.h>
#include <units/angle.h>
#include <units/dimensionless.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/TimedRobot.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <pathplanner/lib/config/PIDConstants.h>


namespace OIConstants {

  inline constexpr int DriverControllerPort = 0;
  inline constexpr int OperatorControllerPort = 1;
}  // namespace OperatorConstants

namespace AlgaeConstants {
  
}

namespace ElevatorConstants
{
  inline constexpr double kP = 0.1;
  inline constexpr double kI = 0;
  inline constexpr double kD = 0;

  inline constexpr auto kS = 1_V;
  inline constexpr auto kG = 1_V;
  inline constexpr auto kV = 0.5_V * 1_s / 1_rad;
  inline constexpr auto kA = 0.1_V * 1_s * 1_s / 1_rad;

  inline constexpr auto kL2 = -24_tr;
  inline constexpr auto kL3 = -62.5_tr;
  inline constexpr auto kL4 = -127_tr;
  inline constexpr auto kL2_Algae = -50_tr;
  inline constexpr auto kL3_Algae = -90_tr;
  inline constexpr auto kReady = -30_tr;
  inline constexpr auto kCollect = -14.5_tr;

  //Change this to calibrate L4 scoring
  inline constexpr auto kCanScore = 0.35;
  
}

namespace ClawConstants
{
  inline constexpr double kP = 0.1;
  inline constexpr double kI = 0;
  inline constexpr double kD = 0;

  inline constexpr auto kScore = -26_tr;
  inline constexpr auto kCollect = -55.5_tr;
  inline constexpr auto kPath = -7_tr;
}

namespace SwerveDriveConstants
{
  inline constexpr pathplanner::PIDConstants translationPID (1, 0, 0.0);
  inline constexpr pathplanner::PIDConstants rotationPID (4, 0, 0.0);
  
  inline units::scalar_t LowerGearDivider {2};

  constexpr units::meter_t DriveWheelDiameterMeters{4.0_in};
  constexpr units::dimensionless_t DriveGearRatio{6.75};

  constexpr units::meters_per_second_t DRIVE_VELOCITY_CONVERSION{
  (units::constants::pi * DriveWheelDiameterMeters) / (DriveGearRatio * 60_s) };

  constexpr units::meter_t DRIVE_POSITION_CONVERSION{
  (units::constants::pi * DriveWheelDiameterMeters) / DriveGearRatio};

  inline constexpr auto kTrackWidth = 21.875_in;
  inline constexpr auto kWheelBase = 26.5_in;
  inline constexpr auto DriveBaseRadius = 17.375_in;

  // If you call DriveSubsystem::Drive with a different period make sure to update
  // this.
  inline constexpr units::second_t kDrivePeriod = frc::TimedRobot::kDefaultPeriod;

  //Trapezoidal Motion
  inline constexpr auto Max_Translation_Speed = 15_fps; // got speeds from swereve drive specialties; so they are only theoretical. https://www.swervedrivespecialties.com/products/mk4i-swerve-module
  inline constexpr auto Max_RCW_Speed = 3.142_rad_per_s; // 14.5 / Circumference_Of_Base / 2pi
  inline constexpr auto Max_Accel = 3_mps_sq;

  inline constexpr auto Capped_FWD_Speed = 10_fps;
  inline constexpr auto Capped_STR_Speed = 10_fps;
  inline constexpr auto Capped_RCW_Speed = 3.142_rad_per_s;

  inline frc::SwerveDriveKinematics<4> Kinematics{
    frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, -kTrackWidth / 2}
  };
}

namespace ExtenderConstants
{
  const double kP = 1;
  const double kI = 0;
  const double kD = 0;
  const bool kInvert = true;
}
