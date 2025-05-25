#include "subsystems/Pneumatics.h"
#include "frc2/command/FunctionalCommand.h"


Pneumatics::Pneumatics()
{
  pnH.EnableCompressorAnalog(MinPressure, MaxPressure);
  SetName("Pneumatics");
}


