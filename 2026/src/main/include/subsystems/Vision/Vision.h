// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include "LimelightHelpers.h"

#include <frc/geometry/Pose2d.h>
#include "subsystems/Vision/Vision.h"
#include <functional>

class Vision : public frc2::SubsystemBase 
{
private:

  void Periodic() override
  {
    _pose = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(_LimeLightName);

    //TODO we should also check the area of the tag to make sure its close enough
    if(_pose.tagCount > 0)
    {
      _updateVisionMeasurement(_pose.pose, _pose.timestampSeconds);
    }
    
  }

public:
  Vision(std::string LimeLightName) : _LimeLightName{LimeLightName}
  {
    SetName("LimeLight");
  }

  auto getPose() -> frc::Pose2d {return _pose.pose;}
  auto getPoseEstimate() -> LimelightHelpers::PoseEstimate {return _pose;}
  auto isTarget() -> bool {return _pose.tagCount > 0;}
  auto isTargetAreaLargeEnough(double TargetArea) -> bool {return _pose.avgTagArea > TargetArea;}

  auto setPoseWithMegaTag2() -> void {
                                      LimelightHelpers::SetRobotOrientation(
                                        _LimeLightName,
                                        LimelightHelpers::getBotPoseEstimate_wpiBlue(_LimeLightName).pose.Rotation().Degrees().value(),0,0,0,0,0);
                                      }
  
private:
  std::string _LimeLightName;
  LimelightHelpers::PoseEstimate _pose;
  std::function<void(frc::Pose2d, units::time::second_t)> _updateVisionMeasurement;
};