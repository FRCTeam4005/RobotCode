
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/PneumaticHub.h>
#include <frc/DoubleSolenoid.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/Compressor.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace CANConstants;

class Pneumatics: public frc2::SubsystemBase {
 public:
  Pneumatics();

  frc::DoubleSolenoid GetDoubleSolenoid(int fwd, int rev)
  {
    return pnH.MakeDoubleSolenoid(fwd,rev);
  }

  void Periodic() override
  {}

 private:
  frc::PneumaticHub pnH{33};
  const units::pressure::pounds_per_square_inch_t MinPressure{80};
  const units::pressure::pounds_per_square_inch_t MaxPressure{95};
};