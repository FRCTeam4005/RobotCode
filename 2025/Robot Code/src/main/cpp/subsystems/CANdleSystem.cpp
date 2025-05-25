// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <iostream>
#include "subsystems/CANdleSystem.h"
#include "ctre/phoenix/led/ColorFlowAnimation.h"
#include "ctre/phoenix/led/FireAnimation.h"
#include "ctre/phoenix/led/LarsonAnimation.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include "ctre/phoenix/led/RgbFadeAnimation.h"
#include "ctre/phoenix/led/SingleFadeAnimation.h"
#include "ctre/phoenix/led/StrobeAnimation.h"
#include "ctre/phoenix/led/TwinkleAnimation.h"
#include "ctre/phoenix/led/TwinkleOffAnimation.h"

using namespace ctre::phoenix::led;

CANdleSystem::CANdleSystem() {
    CANdleConfiguration configAll {};
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType::RGB;
    configAll.brightnessScalar = 1;
    configAll.vBatOutputMode = VBatOutputMode::Modulated;
    m_candle.ConfigAllSettings(configAll, 100);
}


void CANdleSystem::SetLed(int r, int g, int b) {
    m_candle.SetLEDs(r*255, g*255, b*255);
}

frc2::CommandPtr CANdleSystem::SetRed() {
    return frc2::cmd::RunOnce([this]{SetLed(1, 0, 0);}, {this});
}
frc2::CommandPtr CANdleSystem::SetGreen() {
    return frc2::cmd::RunOnce([this]{SetLed(0, 1, 0);}, {this});
}
frc2::CommandPtr CANdleSystem::SetBlue() {
    return frc2::cmd::RunOnce([this]{SetLed(0, 0, 1);}, {this});
}
frc2::CommandPtr CANdleSystem::SetYellow() {
    return frc2::cmd::RunOnce([this]{SetLed(1, 1, 0);}, {this});
}
frc2::CommandPtr CANdleSystem::SetPurple() {
    return frc2::cmd::RunOnce([this]{SetLed(1, 0, 1);}, {this});
}
frc2::CommandPtr CANdleSystem::SetCyan() {
    return frc2::cmd::RunOnce([this]{SetLed(0, 1, 1);}, {this});
}
frc2::CommandPtr CANdleSystem::SetWhite() {
    return frc2::cmd::RunOnce([this]{SetLed(1, 1, 1);}, {this});
}