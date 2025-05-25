#pragma once

#ifndef ROBOT_CONSTANTS_H
#define ROBOT_CONSTANTS_H


#include <frc/MathUtil.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/constants.h>
#include <units/dimensionless.h>

const uint8_t DriverControllerPort = 0;
const uint8_t OperatorControllerPort = 1;

constexpr units::meter_t DriveWheelDiameterMeters{4.0_in - 2_cm};
constexpr units::dimensionless_t GearRatio{6.75};
constexpr units::meters_per_second_t VELOCITY_CONVERSION{
    (units::constants::pi * DriveWheelDiameterMeters) / (GearRatio * 60_s) 
};
constexpr units::meter_t POSITION_CONVERSION{
    (units::constants::pi * DriveWheelDiameterMeters) / GearRatio
};
const int PIGEON_ID = 45;

//Front Left Swerve Module
const bool FL_DRIVE_INVERT{true};
const double FL_DRIVE_KP{.06};
const int FL_DRIVE_ID{61};

const  int FL_TURN_ID{60};
const  bool FL_TURN_INVERT{false};
const  int FL_CANCODER{52};

//Front Right Swerve Module
const  int FR_DRIVE_ID{55} ;
const  bool FR_DRIVE_INVERT{true};
const  double FR_DRIVE_KP{.06};

const  int FR_TURN_ID{54};
const  bool FR_TURN_INVERT{false};
const  int FR_CANCODER{51};
const  double FR_cancoder_offset{0};

//Back Left Swerve Module
const  int BL_DRIVE_ID{59};
const  bool BL_DRIVE_INVERT{true};
const  double BL_DRIVE_KP{.06};

const  int BL_TURN_ID{58};
const  bool BL_TURN_INVERT{false};
const  int BL_CANCODER{53};

//Back Right Swerve Module
const  int BR_DRIVE_ID{57};
const  bool BR_DRIVE_INVERT{true};
const  double BR_DRIVE_KP{.06};

const  int BR_TURN_ID{56};
const  bool BR_TURN_INVERT{false};
const  int BR_CANCODER{50};

//Trapezoidal Motion
const units::meters_per_second_t  Max_FWD_Speed{14.5_fps}; // got speeds from swereve drive specialties; so they are only theoretical. https://www.swervedrivespecialties.com/products/mk4i-swerve-module
const units::meters_per_second_t  Max_STR_Speed{14.5_fps};
const units::radians_per_second_t Max_RCW_Speed{units::constants::pi.value()}; // 14.5 / Circumference_Of_Base / 2pi
const units::meters_per_second_squared_t Max_Accel{3};

const units::meters_per_second_t  Capped_FWD_Speed{3_mps};
const units::meters_per_second_t  Capped_STR_Speed{3_mps};
const units::radians_per_second_t Capped_RCW_Speed{units::constants::pi.value()};

//Forward Gains
const double FWD_P_Gain{0.0};
const double FWD_I_Gain{0.0};
const double FWD_D_Gain{0.0};

//Strafe Gains
const double STR_P_Gain{0.0};
const double STR_I_Gain{0.0};
const double STR_D_Gain{0.0};

//Rotational Gains Velocity Based
const double Vel_RCW_P_Gain{2};
const double Vel_RCW_I_Gain{0.0};
const double Vel_RCW_D_Gain{0.00};

//Rotational Gains Position Based
const double Pos_RCW_P_Gain{1.4};
const double Pos_RCW_I_Gain{0.0};
const double Pos_RCW_D_Gain{0.0032};
// const double Pos_RCW_P_Gain{1};
// const double Pos_RCW_D_Gain{0.32};
#endif