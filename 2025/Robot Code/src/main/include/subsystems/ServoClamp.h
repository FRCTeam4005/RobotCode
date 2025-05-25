#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include "frc/Servo.h"
#include <memory.h>

class WinchServo : public frc2::SubsystemBase
{
public:
  WinchServo();
  frc2::CommandPtr UnlockWinch();
  frc2::CommandPtr LockWinch();

private:
  std::unique_ptr<frc::Servo> Servo_;
  
};