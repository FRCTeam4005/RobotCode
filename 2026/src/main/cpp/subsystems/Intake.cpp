#include "subsystems/Intake.h"

using namespace CANConstants;

Intake::Intake()
{

    IntakeMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kIntakeMotorID);
    ConveyorMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(kConveyorMotorID);
    SetName ("Intake");

}

frc2::CommandPtr Intake::Collect()
{
    return frc2::FunctionalCommand(
        [this] {},
        [this] {SpinIntakeIn();},
        [this] (bool interrupted){StopIntake();},
        [this] {return false;},
        {this}
    ).ToPtr();
}

frc2::CommandPtr Intake::Unstick()
{
    return frc2::FunctionalCommand(
        [this] {},
        [this] {SpinIntakeOut();},
        [this] (bool interrupted){StopIntake();},
        [this] {return false;},
        {this}
    ).ToPtr();
}

void Intake::SpinIntakeOut()
{
    IntakeMotor->Set(1.0);
    ConveyorMotor->Set(-1.0);
}

void Intake::SpinIntakeIn()
{
    IntakeMotor->Set(-1.0);
    ConveyorMotor->Set(1.0);
}

void Intake::StopIntake()
{
    IntakeMotor->Set(0.0);
    ConveyorMotor->Set(0.0);
}