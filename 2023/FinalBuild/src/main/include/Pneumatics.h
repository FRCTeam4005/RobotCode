#ifndef PNEUMATICS_H
#define PNEUMATICS_H

#include <frc/PneumaticHub.h>
#include <frc/DoubleSolenoid.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/Compressor.h>

class Pneumatics
{
public:
  Pneumatics();

  frc::PneumaticHub pnH{33};

  void WristDown()
  {
    WristSolenoid.Set(frc::DoubleSolenoid::kForward);
  }
  
  void WristUp()
  {
    WristSolenoid.Set(frc::DoubleSolenoid::kReverse);
  }

  void GrabberClose()
  {
    Grabber.Set(frc::DoubleSolenoid::kForward);
  }

  void GrabberOpen()
  {
    Grabber.Set(frc::DoubleSolenoid::kReverse);
  }

  void ArmLockClose()
  {
    ArmLock.Set(frc::DoubleSolenoid::kReverse);
  }

  void ArmLockOpen()
  {
    ArmLock.Set(frc::DoubleSolenoid::kForward);
  }

private:
  double scale = 250, offset = -25;
  frc::AnalogPotentiometer pressureSensor{5, scale, offset};
  const units::pressure::pounds_per_square_inch_t MinPressure{20};
  const units::pressure::pounds_per_square_inch_t MaxPressure{30};

  frc::DoubleSolenoid WristSolenoid = pnH.MakeDoubleSolenoid(8,9);
  frc::DoubleSolenoid Grabber = pnH.MakeDoubleSolenoid(0,1);
  frc::DoubleSolenoid ArmLock = pnH.MakeDoubleSolenoid(2,3);

};

#endif