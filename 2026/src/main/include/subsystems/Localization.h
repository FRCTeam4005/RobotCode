// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include "lib/LimelightHelpers.h"

#include <frc/geometry/Pose2d.h>
#include <frc2/RobotModeTriggers.h>
#include "subsystems/Localization.h"
#include <functional>

using namespace LimelightHelpers;
using Pose2d = frc::Pose2d;

class Localization : public frc2::SubsystemBase 
{
private:

  void Periodic() override
  {
    auto CameraPose = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(_LimeLightName);
    //TODO we should also check the area of the tag to make sure its close enough
    if(CameraPose.tagCount <= 0 /* && CameraPose.avgTagArea*/)
    {
      _updateVisionMeasurement(CameraPose.pose,units::time::second_t{CameraPose.latency});
    }
  }

public:
  
  Localization(std::string CameraName, std::function<Pose2d()> getRobotOdometryPose, std::function<void(frc::Pose2d pose, units::time::second_t)> updateVisionMeasurement)
  :  _LimeLightName{CameraName},
  getPose{getRobotOdometryPose}, 
  _updateVisionMeasurement{updateVisionMeasurement}
  {
    frc2::RobotModeTriggers::Disabled().WhileTrue();
    SetName("Localization");
  }

  
  auto getPoseEstimate() -> PoseEstimate {return _pose;}
  auto setPoseWithMegaTag1() -> void {SetRobotOrientation(_LimeLightName,getMegaTag1Yaw().value(),0,0,0,0,0);}
  
  frc::Pose2d getPose() {return _getPose();}
private:
  std::string _LimeLightName;
  PoseEstimate _pose;
  std::function<void(Pose2d, units::time::second_t)> _updateVisionMeasurement;
  std::function<Pose2d()> _getPose;
  

  auto getMegaTag1Yaw() -> units::degree_t {return getBotPoseEstimate_wpiBlue(_LimeLightName).pose.Rotation().Degrees();}
};