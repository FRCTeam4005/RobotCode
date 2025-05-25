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
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#define CompBot 1
// #define PracticeBot 1
#define BOT CompBot


#if (BOT == CompBot)
    #include "RobotsConstants/CompBot.h"
#elif(BOT == PracticeBot)
    #include "RobotsConstants/PracticeBot.h"
//  #pragma message "PRACTICE BOT ARE YOU SURE YOU WANT PRACTICE BOT!!!!!!!!!!"
#else
#endif

namespace OIConstants {

  inline constexpr int DriverControllerPort = 0;
  inline constexpr int OperatorControllerPort = 1;
  inline constexpr int NoteSensorPort = 1;
}  // namespace OperatorConstants

namespace ArmConstants
{
  inline constexpr double kP = 0.1;
  inline constexpr double kI = 0;
  inline constexpr double kD = 0;

  inline constexpr auto kS = 1_V;
  inline constexpr auto kG = 1_V;
  inline constexpr auto kV = 0.5_V * 1_s / 1_rad;
  inline constexpr auto kA = 0.1_V * 1_s * 1_s / 1_rad;

  inline constexpr auto kMaxVelocity = 0.25_rad_per_s;
  inline constexpr auto kMaxAcceleration = 0.5_rad / (1_s * 1_s);

  inline constexpr auto kArmOffset = 0.5_rad;

  inline constexpr auto kAboveBumperPos = 6_tr;
  inline constexpr auto kBelowBumperPos = -5.5_tr;
  inline constexpr auto kSetOnSwervePos = 1_tr;
  inline constexpr auto kClimbPos = 30_tr;
  
}

namespace SwerveDriveConstants
{
  inline units::scalar_t LowerGearDivider {2};

  constexpr units::meter_t DriveWheelDiameterMeters{4.0_in};
  constexpr units::dimensionless_t DriveGearRatio{6.75};

  constexpr units::meters_per_second_t DRIVE_VELOCITY_CONVERSION{
  (units::constants::pi * DriveWheelDiameterMeters) / (DriveGearRatio * 60_s) };

  constexpr units::meter_t DRIVE_POSITION_CONVERSION{
  (units::constants::pi * DriveWheelDiameterMeters) / DriveGearRatio};

  inline constexpr auto kTrackWidth = 22.25_in;
  inline constexpr auto kWheelBase = 27_in;
  inline constexpr auto DriveBaseRadius = 17.375_in;

  // If you call DriveSubsystem::Drive with a different period make sure to update
  // this.
  inline constexpr units::second_t kDrivePeriod = frc::TimedRobot::kDefaultPeriod;

  //Trapezoidal Motion
  inline constexpr auto Max_Translation_Speed = 15_fps; // got speeds from swereve drive specialties; so they are only theoretical. https://www.swervedrivespecialties.com/products/mk4i-swerve-module
  inline constexpr auto Max_RCW_Speed = 3.142_rad_per_s; // 14.5 / Circumference_Of_Base / 2pi
  inline constexpr auto Max_Accel = 3_mps_sq;

  inline constexpr auto Capped_FWD_Speed = 15_fps;
  inline constexpr auto Capped_STR_Speed = 15_fps;
  inline constexpr auto Capped_RCW_Speed = 3_rad_per_s;

  inline constexpr pathplanner::PIDConstants TranslationPID{8, 2, 0.0};
  inline constexpr pathplanner::PIDConstants RotationPID{4, 0, 0.0};

  constexpr inline pathplanner::HolonomicPathFollowerConfig SweveAutoConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
  TranslationPID, // Translation PID constants
  RotationPID, // Rotation PID constants
  Max_Translation_Speed, // Max module speed, in m/s
  DriveBaseRadius,
  pathplanner::ReplanningConfig());

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

namespace LimeLightConstants
{
  const char DefaultName[] = "limelight";


  namespace PipelineNames
  {
    const uint8_t PoseEstimator = 0;
    const uint8_t HorizontalAiming = 2;
  }
}