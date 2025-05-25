// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix/led/CANdle.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/Commands.h>
#include "RobotConstants.h"

class CANdleSystem : public frc2::SubsystemBase {
    ctre::phoenix::led::CANdle m_candle {candleID, ""};
    int LedCount = 100;

 public:
    CANdleSystem();
    auto SetRed() -> frc2::CommandPtr;
    auto SetGreen() -> frc2::CommandPtr;
    auto SetBlue() -> frc2::CommandPtr;
    auto SetYellow() -> frc2::CommandPtr;
    auto SetCyan() -> frc2::CommandPtr;
    auto SetPurple() -> frc2::CommandPtr;
    auto SetFire() -> frc2::CommandPtr;
    auto SetLarson() -> frc2::CommandPtr;
    auto SetWhite() -> frc2::CommandPtr;
    auto SetColorFlow() -> frc2::CommandPtr;

private:
    auto SetLed(int r, int g, int b) -> void;
};