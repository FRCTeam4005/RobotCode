#ifndef COMPRESSOR_H
#define COMPRESSOR_H

#include <frc/PneumaticHub.h>
#include <frc/DoubleSolenoid.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/Compressor.h>
#include "generated/TunerConstants.h"

class Compressor
{
public:
  Compressor();
  
private:
  frc::PneumaticHub pnH{CANConstants::kPneumaticHub};
};

#endif
