#include "subsystems/Intake.h"

Intake::Intake()
{

}

frc2::CommandPtr Intake::FuelOut()
{
    return frc2::FunctionalCommand(
        [this] {},
        [this] {},
        [this] (bool interrupted){},
        [this] {return false;},
        {this}
    ).ToPtr();
}