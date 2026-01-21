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
#include <ctre/phoenix6/CANcoder.hpp>

const uint8_t DriverControllerPort = 0;
const uint8_t OperatorControllerPort = 1;

inline const int PIGEON_ID{45};

//Front Left Swerve Module
inline const int FL_DRIVE_ID{7};
inline const int FL_TURN_ID{8};
inline const int FL_CANCODER{52};

//Front Right Swerve Module
inline const int FR_DRIVE_ID{1};
inline const int FR_TURN_ID{2};
inline const int FR_CANCODER{50};

//Back Left Swerve Module
inline const int BL_DRIVE_ID{5};
inline const int BL_TURN_ID{6};
inline const int BL_CANCODER{53};

//Back Right Swerve Module
inline const int BR_DRIVE_ID{3};
inline const int BR_TURN_ID{4};
inline const int BR_CANCODER{51};

inline const int PDH_ID {30};

    //Front Left Swerve Module
inline constexpr bool FL_DRIVE_INVERT{false};
inline constexpr double FL_DRIVE_KP{.06};
inline constexpr bool FL_TURN_INVERT{true};
inline constexpr units::turn_t FL_cancoder_offset{-0.984375};

//Front Right Swerve Module
inline constexpr bool FR_DRIVE_INVERT{true};
inline constexpr double FR_DRIVE_KP{.06};
inline constexpr bool FR_TURN_INVERT{true};
inline constexpr units::turn_t FR_cancoder_offset{-0.947265625};

//Back Left Swerve Module
inline constexpr bool BL_DRIVE_INVERT{false};
inline constexpr double BL_DRIVE_KP{.06};
inline constexpr bool BL_TURN_INVERT{true};
inline constexpr units::turn_t BL_cancoder_offset{-0.144287109375};

//Back Right Swerve Module
inline constexpr bool BR_DRIVE_INVERT{true};
inline constexpr double BR_DRIVE_KP{.06};
inline constexpr bool BR_TURN_INVERT{true};
inline constexpr units::turn_t BR_cancoder_offset{-0.863037109375};

inline constexpr units::turn_t MagnetSensorRange{ 1_tr };  

inline ctre::phoenix6::signals::SensorDirectionValue FL_Cancoder_Magnet_sensor_Direction = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
inline ctre::phoenix6::signals::SensorDirectionValue FR_Cancoder_Magnet_sensor_Direction = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
inline ctre::phoenix6::signals::SensorDirectionValue BL_Cancoder_Magnet_sensor_Direction = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
inline ctre::phoenix6::signals::SensorDirectionValue BR_Cancoder_Magnet_sensor_Direction = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
//inline ctre::phoenix6::signals::FeedbackSensorSourceValue asdfsadf = ctre::phoenix6::


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