#include "subsystems/IntakeFrontRoller.h"

IntakeFrontRoller::IntakeFrontRoller()
{
  IntakeFrontRollerMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kIntakeMotorID);
  IntakeFrontRollerMotor->Set(0);
  SetName("IntakeFrontRoller");
}

void IntakeFrontRoller::setSpeed(double speed)
{
  IntakeFrontRollerMotor->Set(-speed);
}

frc2::CommandPtr IntakeFrontRoller::Out()
{
  return frc2::FunctionalCommand(
      [this] {},
      [this] {setSpeed(-1);},
      [this] (bool interrupted) {setSpeed(0);},
      [this] {return false;},
      {this}
  ).ToPtr();
}

frc2::CommandPtr IntakeFrontRoller::In()
{
  return frc2::FunctionalCommand(
    [this] {},
    [this] {setSpeed(.5);},
    [this] (bool interrupted) {setSpeed(0);},
    [this] {return false;},
    {this}
  ).ToPtr();
}

frc2::CommandPtr IntakeFrontRoller::Stop()
{
  return this->RunOnce(
    [this] {setSpeed(0);}
  );
}
