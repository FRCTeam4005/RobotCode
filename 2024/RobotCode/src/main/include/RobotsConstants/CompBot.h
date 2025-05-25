#pragma once
#include "Constants.h"


namespace CANConstants {
    inline const int kIntakeMotorID{62};

    inline const int kShooterUpperMotorID{10};
    inline const int kShooterLowerMoterID{9};

    inline const int kArmRotRightID{20};
    inline const int kArmRotLeftID{21};

    inline const int kClimberLeftID{30};    //Change/Remove this
    inline const int kClimberRightID{31};   //Change/Remove this

    inline const int PIGEON_ID{45};
 
    inline const int FL_CANCODER{52};
    inline const int FL_DRIVE_ID{7};
    inline const int FL_TURN_ID{8};

    inline const int FR_CANCODER{51};
    inline const int FR_DRIVE_ID{1};
    inline const int FR_TURN_ID{2};

    inline const int BL_CANCODER{53};
    inline const int BL_DRIVE_ID{5};
    inline const int BL_TURN_ID{6};

    inline const int BR_CANCODER{50};
    inline const int BR_DRIVE_ID{3};
    inline const int BR_TURN_ID{4};

    inline const int PDH_ID {30};
}

namespace SwerveModuleConstants
{
    //Front Left Swerve Module
    inline constexpr bool FL_DRIVE_INVERT{false};
    inline constexpr double FL_DRIVE_KP{.06};
    inline constexpr bool FL_TURN_INVERT{true};
    inline constexpr double FL_cancoder_offset{0.003906};
    
    //Front Right Swerve Module
    inline constexpr bool FR_DRIVE_INVERT{true};
    inline constexpr double FR_DRIVE_KP{.06};
    inline constexpr bool FR_TURN_INVERT{true};
    inline constexpr double FR_cancoder_offset{0.385009765625};
    
    //Back Left Swerve Module
    inline constexpr bool BL_DRIVE_INVERT{false};
    inline constexpr double BL_DRIVE_KP{.06};
    inline constexpr bool BL_TURN_INVERT{true};
    inline constexpr double BL_cancoder_offset{0.851806640625};
    
    //Back Right Swerve Module
    inline constexpr bool BR_DRIVE_INVERT{true};
    inline constexpr double BR_DRIVE_KP{.06};
    inline constexpr bool BR_TURN_INVERT{true};
    inline constexpr double BR_cancoder_offset{0.79052734375};

    inline constexpr bool FL_Cancoder_Magnet_sensor_Direction{false};
    inline constexpr bool FR_Cancoder_Magnet_sensor_Direction{false};
    inline constexpr bool BL_Cancoder_Magnet_sensor_Direction{false};
    inline constexpr bool BR_Cancoder_Magnet_sensor_Direction{false};    

    inline ctre::phoenix6::signals::AbsoluteSensorRangeValue MagnetSensorRange = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1;
}

namespace ShooterConstants
{
    const double kTopShooterFF = 0.12;
    const double kTopShooterP = 0.0;
    const double kTopShooterI = 0.0;
    const double kTopShooterD = 0.0;

    const double kBottomShooterFF = 0.12;
    const double kBottomShooterP = 0.00;
    const double kBottomShooterI = 0.0;
    const double kBottomShooterD = 0.0;
}

// namespace IntakeConstant
// {
//     units::turn_t ToFloorPosition = 0_tr;
// }