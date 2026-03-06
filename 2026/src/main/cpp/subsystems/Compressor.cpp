#include "subsystems/Compressor.h"

Compressor::Compressor()
{
  pnH.EnableCompressorAnalog( units::pounds_per_square_inch_t{80}, //Min On Pressure
                              units::pounds_per_square_inch_t{95}); //Max Off Pressure
}
