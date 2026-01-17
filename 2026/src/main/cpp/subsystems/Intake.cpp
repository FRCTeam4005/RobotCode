#include "subsystems/Intake.h"

Intake::Intake()
{
    IntakeMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kIntakeMotorID);
    SetName("Intake");
}

void Intake::setSpeed(double speed)
{
    IntakeMotor->Set(speed);
}

frc2::Trigger Intake::IsIntakeFull()
{
    return frc2::Trigger{ [this] {return !FuelSensor.Get();}};
}

frc2::CommandPtr Intake::FuelOut()
{
    return frc2::FunctionalCommand(
        [this] {},
        [this] {setSpeed(1);},
        [this] (bool interrupted)
        {
            setSpeed(0);
        },
        [this] {return !IsIntakeFull().Get();},
        {this}
    ).ToPtr();
}

frc2::CommandPtr Intake::FuelUp()
{
  return frc2::FunctionalCommand(
    [this] {},
    [this] {setSpeed(0.5);},
    [this] (bool interrupted){setSpeed(0);},
    [this] {return IsIntakeFull().Get();},
    {this}
  ).ToPtr();
}

frc2::CommandPtr Intake::Stop()
{
  return this->RunOnce(
    [this] {setSpeed(0);}
  );
}