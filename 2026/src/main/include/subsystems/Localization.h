#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "subsystems/Vision/Vision.h"
#include <functional>
#include "subsystems/Vision/LimelightHelpers.h"
#include <units/time.h>

class Localization : public frc2::SubsystemBase
{
public:
  Localization(Vision* Camera, std::function<void()> getPose, std::function<void(frc::Pose2d pose, units::time::second_t)> updateVisionMeasurement)
  : getPose{getPose}, updateVisionMeasurement{updateVisionMeasurement}{}


private:

  void Periodic() override
  {
    auto poseEstimate =  Camera->getPoseEstimate();
    auto pose =  &poseEstimate.pose;
    auto latency =  &poseEstimate.latency;
    auto targetCount =  &poseEstimate.tagCount;
    
    if (*targetCount > 0 /*&& Camera.TargetArea > [Some Value]*/) 
    {
      updateVisionMeasurement(*pose,units::time::second_t{*latency});
    }
  }

  
private:
  std::shared_ptr<Vision> Camera;
  std::function<void()> getPose;
  std::function<void(frc::Pose2d pose, units::time::second_t)> updateVisionMeasurement;
};