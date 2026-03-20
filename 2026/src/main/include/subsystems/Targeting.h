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
    Localization()
    {
        SetName("Targeting");
    }

private:
    void Periodic() override
    {

    }
};