#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <memory.h>
#include "RobotConstants.h"

class Winch : public frc2::SubsystemBase
{
public:
  Winch();
  frc2::CommandPtr In();
  frc2::CommandPtr Out();

private:
  std::unique_ptr<ctre::phoenix6::hardware::TalonFX> Motor_;
};