#ifndef AUTO_LEVEL_BASE
#define AUTO_LEVEL_BASE

#include "Pigeon.h"
#include <units/angle.h>

class RobotLeveling
{
public:

    typedef enum
    {
        TowardOpposingTeam,
        AwayFromOpposingTeam,
    } RobotDirection;

protected:
    bool RobotIsTilted()
    {
        return abs(Pigeon::GetInstance().GetRoll().value()) >= StartingAngle + 1;
    }

    bool RobotIsLevel()
    {
        return abs(Pigeon::GetInstance().GetRoll().value()) <= StartingAngle + 1;
    }

    units::degree_t RobotRoll()
    {
        return Pigeon::GetInstance().GetRoll();
    }

    const units::angle::degree_t TOWARD_OPPOSING_TILT{-11};
    const units::angle::degree_t AWAY_OPPOSING_TILT{11};

    units::angle::degree_t First_Tilt;
    units::angle::degree_t Last_Tilt;
    
    units::velocity::meters_per_second_t climbSpeed;
    units::velocity::meters_per_second_t GroundSpeed;

    double StartingAngle;
    double StartingAngleLower;
    double StartingAngleUpper;
};

#endif