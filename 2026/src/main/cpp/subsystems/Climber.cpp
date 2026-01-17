#include "subsystems/Climber.h"

Climber::Climber()
{
    ClimberMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kClimberID);
    SetName("Climber");
}

void Climber::setSpeed(double speed)
{
    ClimberMotor->Set(speed);
}

frc2::CommandPtr Climber::Up()
{
    return frc2::FunctionalCommand(
        [this] {},
        [this] {setSpeed(1);}, //************************** This probably needs to be changed *********************************
        [this] (bool interrupted)
        {
            setSpeed(0);
        },
        [this] {return false;},
        {this}
    ).ToPtr();
}

frc2::CommandPtr Climber::Down()
{
    return frc2::FunctionalCommand(
        [this] {},
        [this] {setSpeed(-1);}, //************************** This probably needs to be changed *********************************
        [this] (bool interrupted)
        {
            setSpeed(0);
        },
        [this] {return false;},
        {this}
    ).ToPtr();
}

frc2::CommandPtr Climber::Stop()
{
  return this->RunOnce(
    [this] {setSpeed(0);}
  );
}