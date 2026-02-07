#include "subsystems/pneumatics.h"

Pneumatics::Pneumatics()
: m_compressor(34,frc::PneumaticsModuleType::REVPH)
{
  m_compressor.EnableAnalog(units::pounds_per_square_inch_t{80}, //Min On Pressure
                              units::pounds_per_square_inch_t{95}); //Max Off Pressure)
}