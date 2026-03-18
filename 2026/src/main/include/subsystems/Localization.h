// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc2/command/SubsystemBase.h>
#include "lib/LimelightHelpers.h"
#include <frc/geometry/Pose2d.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/DriverStation.h>
#include <functional>
#include <string>
#include <wpi/print.h>

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
        std::function<void(Pose2d, units::time::second_t)> updateVisionMeasurement)
        : _limelightName{cameraName},
          _imu{imu},
          _getPose{getRobotOdometryPose},
          _setOdometryPose{setRobotOdometryPose},
          _updateVisionMeasurement{updateVisionMeasurement}
    {
        SetName("Localization");

        // Set IMU mode once at init — external IMU fused with MegaTag2
        LimelightHelpers::SetIMUMode(_limelightName, 4);
    }

    frc::Pose2d getPose() { return _getPose(); }

private:
    void Periodic() override
    {
        // If disabled, try to seed pose from vision every loop
        if (frc::DriverStation::IsDisabled())
        {
            _TrySeedPoseFromVision();
            return;
        }

        // Guard: don't use vision if gyro is spinning too fast (MegaTag2 unreliable)
        double angularVelocityDegPerSec = std::abs(
            _imu.GetAngularVelocityZWorld().GetValue().value());
        if (angularVelocityDegPerSec > 720.0)
            return;

        // Update Limelight with current robot heading for MegaTag2
        LimelightHelpers::SetRobotOrientation(
            _limelightName,
            _getPose().Rotation().Degrees().value(),
            0, 0, 0, 0, 0);

        // Get MegaTag2 estimate (single call, cached)
        auto estimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(_limelightName);

        // Require at least one tag and a minimum tag area for reliability
        if (estimate.tagCount == 0 || estimate.avgTagArea < 0.25)
            return;

        if (!_hasSeenAprilTag)
        {
            // First time seeing a tag — hard reset odometry to vision pose
            _SeedPoseFromMegaTag1();
            _hasSeenAprilTag = true;
            return;
        }

        // Normal operation — feed vision measurement with correct timestamp
        _updateVisionMeasurement(
            estimate.pose,
            units::time::second_t{estimate.timestampSeconds});
    }

    // Hard-resets odometry using MegaTag1 yaw + MegaTag2 pose
    // Used on first tag acquisition and while disabled
    void _SeedPoseFromMegaTag1()
    {
        // Get yaw from MegaTag1 (more reliable for initial heading)
        auto mt1Estimate = LimelightHelpers::getBotPoseEstimate_wpiBlue(_limelightName);
        if (mt1Estimate.tagCount == 0)
            return;

        double yawDeg = mt1Estimate.pose.Rotation().Degrees().value();
        LimelightHelpers::SetRobotOrientation(_limelightName, yawDeg, 0, 0, 0, 0, 0);

        // Now get MegaTag2 pose with corrected orientation and hard-set odometry
        auto mt2Estimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(_limelightName);
        if (mt2Estimate.tagCount == 0)
            return;

        _setOdometryPose(mt2Estimate.pose);
    }

    // Attempts to seed pose from vision while disabled (called every loop when disabled)
    void _TrySeedPoseFromVision()
    {
        auto estimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(_limelightName);
        if (estimate.tagCount == 0 || estimate.avgTagArea < 0.25)
            return;

        _SeedPoseFromMegaTag1();
        _hasSeenAprilTag = true;
    }

    std::string _limelightName;
    ctre::phoenix6::hardware::Pigeon2 &_imu;
    std::function<void(Pose2d, units::time::second_t)> _updateVisionMeasurement;
    std::function<Pose2d()> _getPose;
    std::function<void(Pose2d)> _setOdometryPose;

    bool _hasSeenAprilTag = false;
};