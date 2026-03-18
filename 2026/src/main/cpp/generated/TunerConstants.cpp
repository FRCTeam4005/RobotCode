#include "generated/TunerConstants.h"
#include "subsystems/Drivetrain.h"

subsystems::Drivetrain TunerConstants::CreateDrivetrain()
{
    std::array<double, 3U> cameraStdDevs = {0.7, 0.7, 9999};
    std::array<double, 3U> drivetrainStdDevs = {0.1, 0.1, 0};

    return subsystems::Drivetrain{DrivetrainConstants, 100_Hz, drivetrainStdDevs, cameraStdDevs, FrontLeft, FrontRight, BackLeft, BackRight};
}
 //91-92HZ