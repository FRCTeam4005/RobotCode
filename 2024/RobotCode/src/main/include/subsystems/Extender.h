
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <subsystems/Extender.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Pneumatics.h"

using namespace CANConstants;

class Extender : public frc2::SubsystemBase {
 public:
  Extender(Pneumatics *Pneumatics_Sys);

  void Periodic() override
  {
    #ifdef DEBUG
    auto disableCompress = frc::SmartDashboard::GetBoolean("disable Compressor", false);
    disableCompress ? pnH.DisableCompressor() : pnH.EnableCompressorDigital();
    #endif
  }

  auto SetPastBumper() -> frc2::CommandPtr;
  auto SetIntoFrame() -> frc2::CommandPtr;
  

 private:
  std::unique_ptr<frc::DoubleSolenoid> Extender_Sol;
  void InitSendable(wpi::SendableBuilder& builder);
};