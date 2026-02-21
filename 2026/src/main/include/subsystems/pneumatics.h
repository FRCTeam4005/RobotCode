#ifndef PNEUMATICS_H
#define PNEUMATICS_H

#include <frc/PneumaticHub.h>
#include <frc/DoubleSolenoid.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/Compressor.h>
#include "generated/TunerConstants.h"

class Pneumatics
{
public:
  Pneumatics();
  frc::PneumaticHub pnH{CANConstants::kPneumaticHub};

private:
  double scale = 250, offset = -25;
  frc::AnalogPotentiometer pressureSensor{5, scale, offset};
  const units::pressure::pounds_per_square_inch_t MinPressure{20};
  const units::pressure::pounds_per_square_inch_t MaxPressure{30};
};

#endif
