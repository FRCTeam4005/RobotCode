
#ifndef SWERVE_MODULE_H
#define SWERVE_MODULE_H

#include "rev/CANSparkMax.h"
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/Controller/PIDController.h>
#include "rev/RelativeEncoder.h"
#include "units/velocity.h"
#include "RobotConstants.h"
#include <memory>
#include <ctre/phoenix6/CANcoder.hpp>
#include <units/angle.h>

class SwerveModule
{
public:
  SwerveModule( uint32_t drive_id, 
                bool drive_invert,
                double kP,
                uint32_t turn_id,
                bool turn_invert,
                uint32_t turn_encoder_id)
  {
    Turn_Motor = std::make_unique<rev::CANSparkMax>(turn_id, rev::CANSparkLowLevel::MotorType::kBrushless);
    Drive_Motor = std::make_unique<rev::CANSparkMax>(drive_id, rev::CANSparkLowLevel::MotorType::kBrushless);

    Turn_PID = std::make_unique<frc::PIDController>(0.01, 0, 0);

    Drive_PID = std::make_unique<rev::SparkPIDController>(Drive_Motor->GetPIDController());

    Drive_Encoder = std::make_unique<rev::SparkRelativeEncoder>(Drive_Motor->GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature));
    Drive_Encoder->SetVelocityConversionFactor(VELOCITY_CONVERSION.value());
    Drive_Encoder->SetPositionConversionFactor(POSITION_CONVERSION.value());

    Turn_Encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(turn_encoder_id);

    //stops the robot from continuing to move after control is let go
    //this is sort of important but is not if we are using the drive motor pid
    Drive_Motor->SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    Drive_Motor->SetInverted(drive_invert);
    Drive_PID->SetP(.06, 0);
    Drive_PID->SetI(7.9e-4,0);
    Drive_PID->SetD(0,0);
    Drive_PID->SetFeedbackDevice(*Drive_Encoder);
    Drive_Motor->BurnFlash();

    Turn_Motor->RestoreFactoryDefaults();
    //deters but doesn't entirely stop the turn motors from spinning
    Turn_Motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    Turn_Motor->SetInverted(turn_invert);

    // this stops the swerve wheels from turning more than 180 degrees
    Turn_PID->EnableContinuousInput(0,360);
  }

/**
 * @param velocity -14.5 to 14.5 feet per second 
 * @brief sets the drive motors speed
 * 
 * */ 
  void Set_Drive_Motor_Velocity(units::meters_per_second_t velocity)
  {
    Drive_PID->SetReference(velocity.value(), rev::CANSparkMax::ControlType::kVelocity, 0);
  }

  /**
   * @param percent -1 to 1
   * @brief pretty much set the voltage between -12 to 12 volts behind the scenes
  */
  void Set_Drive_Percentage(double percent)
  {
    Drive_Motor->Set(percent);
  }
  
  /**
   * @brief returns how many meters from zero the wheels have turned
  */
  units::meter_t Get_Drive_Motor_Totalizer()
  {
    return units::meter_t(Drive_Encoder->GetPosition());
  }

  /**
   * @brief causes both the turn and drive motor the spin freely
  */
  void Go_Limp()
  {
    Drive_Motor->Set(0);
    Turn_Motor->Set(0);
  }

  /**
   * @brief causes the drive motor to face zero degrees with respect to the robot.
  */
  void Allign_Motor()
  {
    Turn_Motor->Set(Turn_PID->Calculate(Turn_Encoder->GetAbsolutePosition().GetValueAsDouble(), 0));
  }

  /**
   * @param new_state 
   * @brief takes new state and applies it to the module
  */
  void Set_State(frc::SwerveModuleState new_state)
  {
    units::degree_t Current_Angle {Turn_Encoder->GetPosition().GetValueAsDouble()}; 
    auto state = frc::SwerveModuleState::Optimize(new_state, frc::Rotation2d(Current_Angle));
    Turn_Motor->Set(Turn_PID->Calculate(Current_Angle.value(), state.angle.Degrees().value()));
    Drive_PID->SetReference(state.speed.value(), rev::CANSparkMax::ControlType::kVelocity, 0);
  }

  std::unique_ptr<rev::CANSparkMax> Turn_Motor;
  std::unique_ptr<rev::CANSparkMax> Drive_Motor;

  std::unique_ptr<rev::SparkPIDController>    Drive_PID;
  // we are using frc pid instead of spark max because we cannot connect absolute mag encoders direcly up to the spark max
  // nor can we directly pass the encoder information into the spark max.
  std::unique_ptr<frc::PIDController>           Turn_PID;
  std::unique_ptr<rev::RelativeEncoder>  Drive_Encoder;

  std::unique_ptr<ctre::phoenix6::hardware::CANcoder> Turn_Encoder;
};

#endif