#include "generated/TunerConstants.h"
#include "subsystems/Drivetrain.h"

subsystems::Drivetrain TunerConstants::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
