#pragma once

#include <frc2/command/SubsystemBase.h>
#include "lib/LimelightHelpers.h"
#include <frc/geometry/Pose2d.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/DriverStation.h>
#include <functional>
#include <string>
#include <wpi/print.h>
#include <memory>

using namespace LimelightHelpers;
using Pose2d = frc::Pose2d;

class Localization : public frc2::SubsystemBase
{
public:
    Localization(
        std::string cameraName,
        ctre::phoenix6::hardware::Pigeon2 &imu,
        std::function<Pose2d()> getRobotOdometryPose,
        std::function<void(Pose2d)> setRobotOdometryPose,
        std::function<void(Pose2d, units::time::second_t)> updateVisionMeasurement) : 
          _imu{imu},
          _getPose{getRobotOdometryPose},
          _setOdometryPose{setRobotOdometryPose},
          _updateVisionMeasurement{updateVisionMeasurement}
    {
        SetName("Localization");
    }

    auto addLimeLight(std::string cameraName) -> Localization& 
    {
        // tells the limelight to moosh together the pigeon and the internal IMU
        LimelightHelpers::SetIMUMode(cameraName, 4);
        _limeLightNames.push_back(cameraName);
    }

    auto getPose() -> frc::Pose2d { return _getPose(); }

private:
    void Periodic() override
    {
        if (frc::DriverStation::IsDisabled())
        {
            _TrySeedPoseFromVision();
            return;
        }

        double RotatingToFast = std::abs(_imu.GetAngularVelocityZWorld().GetValue().value()) > 720.0 ;
        if (RotatingToFast) return;


        LimelightHelpers::SetRobotOrientation(
            _limelightName,
            _getPose().Rotation().Degrees().value(),
            0, 0, 0, 0, 0);


        auto estimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(_limelightName);


        auto noTags = estimate.tagCount == 0;
        auto TagsToFarAway = estimate.avgTagArea < 0.25;

        if (noTags || TagsToFarAway) return;

        if (!_hasSeenAprilTag)
        {
            _SeedPoseFromMegaTag1();
            _hasSeenAprilTag = true;
            return;
        }


        _updateVisionMeasurement( estimate.pose, units::time::second_t{estimate.timestampSeconds});
    }

    void _SeedPoseFromMegaTag1(std::string limeLightName)
    {

        auto mt1Estimate = LimelightHelpers::getBotPoseEstimate_wpiBlue(limeLightName);
        if (mt1Estimate.tagCount == 0) return;

        double yawDeg = mt1Estimate.pose.Rotation().Degrees().value();
        LimelightHelpers::SetRobotOrientation(limeLightName, yawDeg, 0, 0, 0, 0, 0);


        auto mt2Estimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(limeLightName);
        if (mt2Estimate.tagCount == 0) return;

        _setOdometryPose(mt2Estimate.pose);
    }

    //only called when robot is disabled
    void _TrySeedPoseFromVision(std::string limeLightName)
    {
        auto estimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(limeLightName);
        if (estimate.tagCount == 0 || estimate.avgTagArea < 0.25) return;

        _SeedPoseFromMegaTag1();
        _hasSeenAprilTag = true;
    }

    std::vector<std::string> _limeLightNames {};
    ctre::phoenix6::hardware::Pigeon2 &_imu;
    std::function<Pose2d()> _getPose;
    std::function<void(Pose2d)> _setOdometryPose;
    std::function<void(Pose2d, units::time::second_t)> _updateVisionMeasurement;

    bool _hasSeenAprilTag = false;
};