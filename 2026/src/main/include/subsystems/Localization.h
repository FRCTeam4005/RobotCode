// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include "lib/LimelightHelpers.h"

#include <frc/geometry/Pose2d.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include "subsystems/Localization.h"
#include <functional>
#include <wpi/print.h>
#include <ctre/phoenix6/Pigeon2.hpp>

using namespace LimelightHelpers;
using Pose2d = frc::Pose2d;

class Localization : public frc2::SubsystemBase 
{
private:

  void Periodic() override
  {
    auto CameraPoseEstimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(_LimeLightName);
    //TODO we should also check the area of the tag to make sure its close enough
    if(CameraPoseEstimate.tagCount > 0  && CameraPoseEstimate.avgTagArea > 0.25)
    {
      if (!_hasSeenAprilTag)
      {
        _SetRobotOrientation_With_MegaTag1();
        CameraPoseEstimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(_LimeLightName);
        _SetOdometryPose(CameraPoseEstimate.pose);
        _hasSeenAprilTag = true;
      }
      else
      {
        LimelightHelpers::SetIMUMode(_LimeLightName, 4);
        SetRobotOrientation(_LimeLightName,getPose().Rotation().Degrees().value(),0,0,0,0,0);
        CameraPoseEstimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(_LimeLightName);
        _updateVisionMeasurement(CameraPoseEstimate.pose,units::time::second_t{CameraPoseEstimate.latency});
      }
    }
  }

public:
  
  Localization(std::string CameraName, ctre::phoenix6::hardware::Pigeon2& IMU, std::function<Pose2d()> getRobotOdometryPose, std::function<void(Pose2d)> setRobotOdometryPose, std::function<void(Pose2d, units::time::second_t)> updateVisionMeasurement):
  _LimeLightName{CameraName},
  _IMU{IMU},
  _getPose{getRobotOdometryPose}, 
  _SetOdometryPose{setRobotOdometryPose}, 
  _updateVisionMeasurement{updateVisionMeasurement}
  {
    SetName("Localization");
  }

  
  auto getPoseEstimate() -> PoseEstimate {return _pose;}
  
  frc::Pose2d getPose() {return _getPose();}
  private:
  std::string _LimeLightName;
  ctre::phoenix6::hardware::Pigeon2& _IMU;
  std::function<void(Pose2d, units::time::second_t)> _updateVisionMeasurement;
  std::function<Pose2d()> _getPose;
  std::function<void(Pose2d)> _SetOdometryPose;
  PoseEstimate _pose;
  bool _hasSeenAprilTag = false;
  
  
  auto _SetRobotOrientation_With_MegaTag1() -> void {SetRobotOrientation(_LimeLightName,_getMegaTag1Yaw().value(),0,0,0,0,0);}
  auto _getMegaTag1Yaw() -> units::degree_t {return getBotPoseEstimate_wpiBlue(_LimeLightName).pose.Rotation().Degrees();}
};