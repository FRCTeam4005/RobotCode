
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <subsystems/Extender.h>

class Climber : public frc2::SubsystemBase 
{
public:
  Climber(Pneumatics *Pneumatics_Sys);
  auto Ascend() -> frc2::CommandPtr;
  auto Descend() -> frc2::CommandPtr;
  auto LockIn() -> frc2::CommandPtr;
  
frc2::CommandPtr Test_Lock_Pawl();
frc2::CommandPtr Test_Unlock_Pawl();

private:
  ctre::phoenix6::controls::PositionVoltage PostionClosedLoop{0_tr, 0_tps, true, 0_V, 0, false};
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> climberMotor;

  std::unique_ptr<frc::DoubleSolenoid> Climber_Pawl_Sol;
 
  void SetClimberSpeed(double speed);
  double GetPosition();
};