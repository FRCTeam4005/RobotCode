#include "subsystems/SwerveModule.h"
#include <rev/SparkMax.h>
#include <rev/config/SparkBaseConfig.h>


// * The steering gear ratio of the MK4i is 150/7:1. https://www.swervedrivespecialties.com/products/mk4i-swerve-module */

SwerveModule::SwerveModule( uint32_t drive_id, 
                            bool drive_invert,
                            double kP,
                            uint32_t turn_id,
                            bool turn_invert,
                            uint32_t turn_encoder_id,
                            const ctre::phoenix6::configs::CANcoderConfiguration& turn_CanCoder_Config )
{
    Turn_Motor_ = std::make_unique<rev::spark::SparkMax>(turn_id, rev::spark::SparkMax::MotorType::kBrushless);
    Drive_Motor_ = std::make_unique<rev::spark::SparkMax>(drive_id, rev::spark::SparkMax::MotorType::kBrushless);

    rev::spark::SparkBaseConfig Turn_Motor_Config_;
    rev::spark::SparkBaseConfig Drive_Motor_Config_;

    Turn_PID_ = std::make_unique<frc::PIDController>(0.01, 0, 0);

    Drive_PID_ = std::make_unique<rev::spark::SparkClosedLoopController>(Drive_Motor_->GetClosedLoopController());

    Drive_Encoder_ = std::make_unique<rev::spark::SparkRelativeEncoder>(Drive_Motor_->GetEncoder());

    Turn_CanCoder_ = std::make_unique<ctre::phoenix6::hardware::CANcoder>(turn_encoder_id);





    //Drive_Motor_Config_->SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    Drive_Motor_Config_.SmartCurrentLimit(40);
    Drive_Motor_Config_.closedLoop.Pid(.06,7.9e-4,0,rev::spark::ClosedLoopSlot::kSlot0);
    Drive_Motor_Config_.Inverted(drive_invert);
    Drive_Motor_Config_.encoder.PositionConversionFactor(SwerveDriveConstants::DRIVE_POSITION_CONVERSION.value());
    Drive_Motor_Config_.encoder.VelocityConversionFactor(SwerveDriveConstants::DRIVE_VELOCITY_CONVERSION.value());
    Drive_Motor_Config_.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder);
    Drive_Motor_Config_.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);



    Turn_PID_->EnableContinuousInput(0,360);
    //Turn_Motor_Config_->SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    Turn_Motor_Config_.SmartCurrentLimit(40);
    Turn_Motor_Config_.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    Turn_Motor_Config_.Inverted(turn_invert);
    //Turn_Motor_Config_.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAlternateOrExternalEncoder);

    Turn_Motor_->Configure(Turn_Motor_Config_, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    Drive_Motor_->Configure(Drive_Motor_Config_, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);


    // //? might calibrate the neo encoder with the cancoders
    // //? to remove some strain on the CANbus bandwidth
    // Turn_Motor_->GetPIDController().SetPositionPIDWrappingEnabled(true);
    // Turn_Motor_->GetPIDController().SetPositionPIDWrappingMinInput(0);
    // Turn_Motor_->GetPIDController().SetPositionPIDWrappingMaxInput(360);
    // Turn_Motor_->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor).SetPosition(Turn_CanCoder_->GetAbsolutePosition().GetValueAsDouble() * 360);

    Turn_CanCoder_->GetConfigurator().Apply(turn_CanCoder_Config);
}

/**
 * @param velocity -14.5 to 14.5 feet per second 
 * @brief sets the drive motors speed
 * 
 * */ 
void SwerveModule::Set_Drive_Motor_Velocity(units::meters_per_second_t velocity)
{
    Drive_PID_->SetReference(velocity.value(), rev::spark::SparkMax::ControlType::kVelocity, rev::spark::ClosedLoopSlot::kSlot0);
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
    auto state = frc::SwerveModuleState(new_state);
    state.Optimize(frc::Rotation2d(currAngle));
    Turn_Motor_->Set(Turn_PID_->Calculate(currAngle.Degrees().value(), state.angle.Degrees().value()));
    Drive_PID_->SetReference(state.speed.value(), rev::spark::SparkMax::ControlType::kVelocity, rev::spark::ClosedLoopSlot::kSlot0);
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



