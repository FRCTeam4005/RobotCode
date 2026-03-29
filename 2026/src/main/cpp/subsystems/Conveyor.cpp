#include "subsystems/Conveyor.h"

IntakeConveyor::IntakeConveyor()
{
  IntakeConveyorMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kConveyorMotorID);
  IntakeConveyorMotor->Set(0);
  SetName("IntakeConveyor");
}

void IntakeConveyor::setSpeed(double speed)
{
  IntakeConveyorMotor->Set(speed);
}

frc2::CommandPtr IntakeConveyor::Out()
{
    return frc2::FunctionalCommand(
        [this] {},
        [this] {setSpeed(-.5);},
        [this] (bool interrupted) {},
        [this] {return true;},
        {this}
    ).ToPtr();
}

frc2::CommandPtr IntakeConveyor::In()
{
  return frc2::FunctionalCommand(
    [this] {},
    [this] {setSpeed(1);},
    [this] (bool interrupted) {},
    [this] {return true;},
    {this}
  ).ToPtr();
}

frc2::CommandPtr IntakeConveyor::Stop()
{
  return this->RunOnce(
    [this] {setSpeed(0);}
  );
}
