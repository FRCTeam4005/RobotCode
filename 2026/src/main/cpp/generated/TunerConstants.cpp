#include "generated/TunerConstants.h"
#include "subsystems/Drivetrain.h"

subsystems::Drivetrain TunerConstants::CreateDrivetrain()
{
    std::array<double, 3U> CamperaThings = {2,2,360};
    std::array<double, 3U> DriveTrainThings = {0.1,0.1,0};

    return subsystems::Drivetrain{DrivetrainConstants, 92_Hz, DriveTrainThings, CamperaThings, FrontLeft, FrontRight, BackLeft, BackRight};
}
 //91-92HZ