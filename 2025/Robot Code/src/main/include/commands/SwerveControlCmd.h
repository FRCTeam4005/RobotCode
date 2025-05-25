#pragma once 

#include <frc2/command/Commands.h>
#include "subsystems/Drivetrain.h"
#include <ctre/phoenix6/Pigeon2.hpp>
class SwerveControlCmd : public frc2::CommandHelper<frc2::Command, SwerveControlCmd>
{
public:

  explicit SwerveControlCmd(Drivetrain&, DriverController&, ctre::phoenix6::hardware::Pigeon2&);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  Drivetrain& subsystem;
  DriverController& DriverControl;
  ctre::phoenix6::hardware::Pigeon2& IMU;
};