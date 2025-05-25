#include "subsystems/Winch.h"

Winch::Winch()
{
    Motor_ = std::make_unique<ctre::phoenix6::hardware::TalonFX>(Climb_Winch);
}

frc2::CommandPtr Winch::In()
{
    return frc2::FunctionalCommand(
    [this] {},
    [this] {Motor_->Set(0.5);},
    [this] (bool interrupted) {Motor_->Set(0);},
    [this] {return false;},
    {}
    ).ToPtr();
}

frc2::CommandPtr Winch::Out()
{
    return frc2::FunctionalCommand(
    [this] {},
    [this] {Motor_->Set(-0.5);},
    [this] (bool interrupted) {Motor_->Set(0);},
    [this] {return false;},
    {}
    ).ToPtr();
}


