#include "subsystems/ServoClamp.h"

WinchServo::WinchServo()
{
    Servo_ = std::make_unique<frc::Servo>(0);
}

frc2::CommandPtr WinchServo::LockWinch()
{
    return frc2::cmd::RunOnce([this]{this->Servo_->Set(1.0);}, {this});
}

frc2::CommandPtr WinchServo::UnlockWinch()
{  
    return frc2::cmd::RunOnce([this]{this->Servo_->Set(0.0);}, {this});
}