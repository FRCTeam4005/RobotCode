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
  
private:
  frc::PneumaticHub pnH{CANConstants::kPneumaticHub};
};

#endif
