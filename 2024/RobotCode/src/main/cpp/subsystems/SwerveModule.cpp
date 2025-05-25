#include "subsystems/SwerveModule.h"
#include <rev/CANSparkMax.h>


// * The steering gear ratio of the MK4i is 150/7:1. https://www.swervedrivespecialties.com/products/mk4i-swerve-module */

SwerveModule::SwerveModule( uint32_t drive_id, 
                            bool drive_invert,
                            double kP,
                            uint32_t turn_id,
                            bool turn_invert,
                            uint32_t turn_encoder_id,
                            const ctre::phoenix6::configs::CANcoderConfiguration& turn_CanCoder_Config )
{
    Turn_Motor_ = std::make_unique<rev::CANSparkMax>(turn_id, rev::CANSparkLowLevel::MotorType::kBrushless);
    Drive_Motor_ = std::make_unique<rev::CANSparkMax>(drive_id, rev::CANSparkLowLevel::MotorType::kBrushless);
    Turn_Motor_->SetSmartCurrentLimit(40);
    Drive_Motor_->SetSmartCurrentLimit(40);

    Turn_PID_ = std::make_unique<frc::PIDController>(0.01, 0, 0);

    Drive_PID_ = std::make_unique<rev::SparkPIDController>(Drive_Motor_->GetPIDController());

    Drive_Encoder_ = std::make_unique<rev::SparkRelativeEncoder>(Drive_Motor_->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
    Drive_Encoder_->SetVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION.value());
    Drive_Encoder_->SetPositionConversionFactor(DRIVE_POSITION_CONVERSION.value());

    Turn_CanCoder_ = std::make_unique<ctre::phoenix6::hardware::CANcoder>(turn_encoder_id);

    //stops the robot from continuing to move after control is let go
    //this is sort of important but is not if we are using the drive motor pid
    Drive_Motor_->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    Drive_Motor_->SetInverted(drive_invert);
    Drive_PID_->SetP(.06, 0);
    Drive_PID_->SetI(7.9e-4,0);
    Drive_PID_->SetD(0,0);
    Drive_PID_->SetFeedbackDevice(*Drive_Encoder_);
    Drive_Motor_->BurnFlash();

    Turn_Motor_->RestoreFactoryDefaults();
    //deters but doesn't entirely stop the turn motors from spinning
    Turn_Motor_->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    Turn_Motor_->SetInverted(turn_invert);

    // this stops the swerve wheels from turning more than 180 degrees
    Turn_PID_->EnableContinuousInput(0,360);

    //? might calibrate the neo encoder with the cancoders
    //? to remove some strain on the CANbus bandwidth
    ////Turn_Motor_->GetPIDController().SetPositionPIDWrappingEnabled(true);
    ////Turn_Motor_->GetPIDController().SetPositionPIDWrappingMinInput(0);
    ////Turn_Motor_->GetPIDController().SetPositionPIDWrappingMaxInput(360);
    ////Turn_Motor_->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor).SetPosition(Turn_CanCoder_->GetAbsolutePosition().GetValueAsDouble() * 360);

    Turn_CanCoder_->GetConfigurator().Apply(turn_CanCoder_Config);
}

/**
 * @param velocity -14.5 to 14.5 feet per second 
 * @brief sets the drive motors speed
 * 
 * */ 
void SwerveModule::Set_Drive_Motor_Velocity(units::meters_per_second_t velocity)
{
    Drive_PID_->SetReference(velocity.value(), rev::CANSparkMax::ControlType::kVelocity, 0);
}

void SwerveModule::Set_Turn_Angle(units::degree_t degree)
{
    auto currAngle = GetPosition().angle;
    Turn_Motor_->Set(Turn_PID_->Calculate(currAngle.Degrees().value(), degree.value()));
}

/**
 * @brief causes both the turn and drive motor the spin freely
 */
void SwerveModule::stop()
{
    Drive_Motor_->Set(0);
    Turn_Motor_->Set(0);
}

/**
 * @param new_state 
 * @brief takes new state and applies it to the module
 */
void SwerveModule::SetDesiredState(const frc::SwerveModuleState& new_state)
{ 
    auto currAngle = GetPosition().angle;
    auto state = frc::SwerveModuleState::Optimize(new_state, frc::Rotation2d(currAngle));
    Turn_Motor_->Set(Turn_PID_->Calculate(currAngle.Degrees().value(), state.angle.Degrees().value()));
    Drive_PID_->SetReference(state.speed.value(), rev::CANSparkMax::ControlType::kVelocity, 0);
}

frc::SwerveModulePosition SwerveModule::GetPosition() 
{
    return {units::meter_t{Drive_Encoder_->GetPosition()}, units::degree_t{Turn_CanCoder_->GetPosition().GetValueAsDouble()*360}};
}

units::degree_t SwerveModule::getTurningPostion()
{
    return Turn_CanCoder_->GetPosition().GetValue();
}

units::meters_per_second_t SwerveModule::getDriveVelocity()
{
    return units::meters_per_second_t{Drive_Encoder_->GetVelocity()};
}

frc::SwerveModuleState SwerveModule::getState()
{
    return frc::SwerveModuleState{getDriveVelocity(), frc::Rotation2d(getTurningPostion())};
}



