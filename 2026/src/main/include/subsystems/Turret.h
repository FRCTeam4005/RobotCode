
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "generated/TunerConstants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <units/angle.h>
#include <units/voltage.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>
#include <iostream>
#include "frc2/command/FunctionalCommand.h"
#include "frc/smartdashboard//SmartDashboard.h"
#include <cmath>
#include "subsystems/Drivetrain.h"
#include <ctre/phoenix6/Pigeon2.hpp>


class Turret : public frc2::SubsystemBase
{
 public:
    Turret();
    auto Move(units::turn_t goal) -> frc2::CommandPtr;
    auto ShootDrivers() -> frc2::CommandPtr;
    auto GetPosition() -> units::angle::turn_t;

private:
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> TurretMotor;
    subsystems::Drivetrain drivetrain{TunerConstants::CreateDrivetrain()};
    std::unique_ptr<ctre::phoenix6::hardware::Pigeon2> Pigeon_Sys;
    ctre::phoenix6::controls::MotionMagicVoltage elevate_mmReq{0_tr};
    units::turn_t position;
    double angle;

    void Periodic () override
    {
        frc::SmartDashboard::PutNumber("Turret Position", GetPosition().value());
        position = GetPosition();
        angle = ((180.0 - Pigeon_Sys->GetYaw().GetValueAsDouble())/360.0)*10.0;
        frc::SmartDashboard::PutNumber("Angle", angle);
    }

    void SetTurretCommand(units::turn_t goal);
};