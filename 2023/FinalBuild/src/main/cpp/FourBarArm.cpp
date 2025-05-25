#include "FourBarArm.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/time.h>
#include <frc/Timer.h>

using namespace ctre::phoenix6;
using namespace units::angle;

FourBarArm::FourBarArm()
{
  configs::TalonFXConfiguration fourBarArmMotorConfig{};
  configs::TalonFXConfiguration carMotorConfig{};

  _4barArmMotor.GetConfigurator().Apply(configs::TalonFXConfiguration()); //this was _4barArmMotor.ConfigFactoryDefault();
  fourBarArmMotorConfig.Slot0.kP = 0.05;
  fourBarArmMotorConfig.Slot0.kI = 0.00002;
  fourBarArmMotorConfig.Slot0.kD = 0.000001;
  _4barArmMotor.GetConfigurator().Apply(fourBarArmMotorConfig);
  _4barArmMotor.SetInverted(false);
  _4barArmMotor.SetNeutralMode(signals::NeutralModeValue::Brake);

  _4barArmMotor.SetPosition(degree_t(0));

  //_4barArmMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 50);
  //_4barArmMotor.SetSelectedSensorPosition(0);
  //_4barArmMotor.SelectProfileSlot(0,0);
  //_4barArmMotor.Config_IntegralZone(0, 500);
  //_4barArmMotor.ConfigClosedLoopPeakOutput(0, 1);
  //_4barArmMotor.ConfigClosedLoopPeriod(0, 1);
  //_4barArmMotor.SetPosition(0);
  //_4barArmMotor.ConfigSupplyCurrentLimit(20,50);


  carMotor.GetConfigurator().Apply(configs::TalonFXConfiguration()); //this was carMotor.ConfigFactoryDefault();
  carMotorConfig.Slot0.kP = 0.06;
  carMotorConfig.Slot0.kI = 0.0001;
  carMotorConfig.Slot0.kD = 0.01;
  carMotor.GetConfigurator().Apply(carMotorConfig);
  carMotor.SetInverted(false);
  carMotor.SetNeutralMode(signals::NeutralModeValue::Brake);

  // carMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 50);
  //carMotor.SetSelectedSensorPosition(0);
  // carMotor.SelectProfileSlot(0,0);
  // carMotor.Config_kF(0, 0);
  // carMotor.Config_IntegralZone(0, 500);
  // carMotor.ConfigClosedLoopPeakOutput(0, 1);
  // carMotor.ConfigClosedLoopPeriod(0,1);
}

void FourBarArm::UpDown(OperatorController &controller)
{
  auto currentPostion = _4barArmMotor.GetPosition().GetValue();
  double ChangeAmount = 3000;

  if(LinearActuatorPosition >= MIDDLE_ARM_POSITION)
  {
    ChangeAmount = 5000;
  }

  // if( (controller.Get4BAUpDown() < -0.05) && (FourBarLowerLimitSwitch.Get()))
  // {
  //   LinearActuatorPosition += ChangeAmount * controller.Get4BAUpDown();
  //   _4barArmMotor.SetPosition(LinearActuatorPosition);
  // }
  // else if( (controller.Get4BAUpDown() > 0.05) && (FourBarUpperLimitSwitch.Get()))
  // {
  //   LinearActuatorPosition += ChangeAmount * controller.Get4BAUpDown();
  //   _4barArmMotor.SetPosition(LinearActuatorPosition);
  // } 


  if((controller.getXButton() > -0.05) && (currentPostion < BOTTOM_ARM_POSITION + turn_t{500}) && (currentPostion > BACK_CARRIAGE_POSITION - turn_t{1000})) 
  {
    _Pneumatics.ArmLockClose();
  } 
  else 
  {
    _Pneumatics.ArmLockOpen();
  } 

  if((LinearActuatorPosition > TOP_ARM_POSITION)) 
  {
    LinearActuatorPosition = TOP_ARM_POSITION;
    _4barArmMotor.SetPosition(LinearActuatorPosition);
  }
  else if((LinearActuatorPosition < BOTTOM_ARM_POSITION) && (!FourBarLowerLimitSwitch.Get())) 
  {
    LinearActuatorPosition = BOTTOM_ARM_POSITION;
    _4barArmMotor.SetPosition(LinearActuatorPosition);
  } 

}

void FourBarArm::InOut(OperatorController &controller)
{
  
  if((controller.getCarriageMovement() > 0.1) && (FrontCarriageLS.Get()))
  {
    CarriagePosition -= turn_t{2100};
    carMotor.SetPosition(CarriagePosition);
  }
  else if( (controller.getCarriageMovement() < -0.1) && (RearCarriageLS.Get()) )
  {
    CarriagePosition += turn_t{2100};
    carMotor.SetPosition(CarriagePosition);
  }
}


void FourBarArm::Intake(OperatorController &controller)
{ 
  //Solenoid Control
  if(controller.getClawExtend()) 
  {
    _Pneumatics.WristDown();
  } 
  else if(controller.getClawRetract()) 
  {
    _Pneumatics.WristUp();
  }

  if(controller.getGrabberButton()) 
  {
    _Pneumatics.GrabberOpen();
  } 
  else 
  {
    _Pneumatics.GrabberClose();
  }
}

void FourBarArm::MovementPresets(OperatorController &controller)
{
  
  switch (controller.FourbaPositionSelect())
  {
  case OperatorController::HIGHEST_4ba_PRESET:
      //carMotor.SelectProfileSlot(1,0);
      _4barArmMotor.SetPosition(TOP_ARM_POSITION);
      carMotor.SetPosition(FRONT_CARRIAGE_POSITION);
      LinearActuatorPosition = TOP_ARM_POSITION;
      if(_4barArmMotor.GetPosition().GetValueAsDouble() >= 40000)
      {
        _Pneumatics.WristDown();
      }
      else
      {
        _Pneumatics.WristUp();
      }

      // frc::SmartDashboard::PutNumber("reading for 4bar Linear Actutator", _4barArmMotor.GetSelectedSensorPosition(0));
    break;

  case  OperatorController::MIDDLE_4ba_PRESET:
      _4barArmMotor.SetPosition(MIDDLE_ARM_POSITION);
      LinearActuatorPosition = MIDDLE_ARM_POSITION;
      carMotor.SetPosition(BACK_CARRIAGE_POSITION);
      _Pneumatics.WristDown();
      // frc::SmartDashboard::PutNumber("reading for 4bar Linear Actutator", _4barArmMotor.GetSelectedSensorPosition(0));
    break;

  case  OperatorController:: LOWEST_4ba_PRESET:
      _4barArmMotor.SetPosition(BOTTOM_ARM_POSITION);
      LinearActuatorPosition = BOTTOM_ARM_POSITION;
      carMotor.SetPosition(BACK_CARRIAGE_POSITION);
      _Pneumatics.WristUp();
      // frc::SmartDashboard::PutNumber("reading for 4bar Linear Actutator", _4barArmMotor.GetSelectedSensorPosition(0));
    break;

  default:
    break;
  }
}

void FourBarArm::AutoInit()
{
  currentState = PLACE_CONE;
  _Pneumatics.GrabberClose();
  autoTimer.Stop();
  autoTimer.Reset();
  _Pneumatics.ArmLockOpen();
}

int FourBarArm::Auto()
{
  switch(currentState)
  {
    case PLACE_CONE:
      if( PlaceCone() > -1 )
      {
        autoTimer.Restart();
        currentState = WAIT_FOR_ARM;
      }
    break;

    case WAIT_FOR_ARM:
      if(autoTimer.Get() >= .5_s)
      {
        currentState = ARM_DOWN;
      }
    break;

    case ARM_DOWN:
      ArmDown();
      currentState = DONE;
    break;

    case DONE:
      autoTimer.Stop();
      autoTimer.Reset();
      return 0;
    break;

    default:
    break;
  }

  return -1;
}

int FourBarArm::PlaceCone()
{
  turn_t Current_Position {_4barArmMotor.GetPosition().GetValue()};

  _4barArmMotor.SetPosition(TOP_ARM_POSITION);
  carMotor.SetPosition(FRONT_CARRIAGE_POSITION);
  LinearActuatorPosition = TOP_ARM_POSITION;
  if(Current_Position.value() >= 45000)
  {
    _Pneumatics.WristDown();
  }

  if(Current_Position.value() >= 252000)
  {
    _Pneumatics.GrabberOpen();
    _Pneumatics.WristUp();
    carMotor.SetPosition(BACK_CARRIAGE_POSITION);
    return 0;
  }

  return -1;
}

void FourBarArm::ArmDown()
{
  _4barArmMotor.SetPosition(BOTTOM_ARM_POSITION);
}
