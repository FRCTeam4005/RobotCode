#ifndef FOURBARARM_H
#define FOURBARARM_H

#include "rev/CANSparkMax.h"
#include "OperatorController.h"
#include <frc/DigitalInput.h>
#include "Pneumatics.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

#include <ctre/phoenix6/TalonFX.hpp>


class FourBarArm
{
public:
  FourBarArm();

  void UpDown(OperatorController &controller);
  void InOut(OperatorController &controller);
  void Intake(OperatorController &controller);
  void MovementPresets(OperatorController &controller);
  int Auto();
  void AutoInit();
  units::angle::turn_t LinearActuatorPosition{BOTTOM_ARM_POSITION};
  units::angle::turn_t CarriagePosition {0};
  
  void PrintLineActuatorPosition()
  {
    frc::SmartDashboard::PutNumber("FourBarARm Position", LinearActuatorPosition.value());
  }

  void TeleopInit()
  {
    LinearActuatorPosition = units::angle::turn_t{0};
    CarriagePosition = units::angle::turn_t{-10};
    _Pneumatics.ArmLockClose();
  }

private:
  frc::Timer autoTimer{};

  ctre::phoenix6::hardware::TalonFX _4barArmMotor{62};
  ctre::phoenix6::hardware::TalonFX carMotor{17};
  typedef enum{
    PLACE_CONE,
    WAIT_FOR_ARM,
    ARM_DOWN,
    DONE
  } states_t;
  states_t currentState = PLACE_CONE;

  int PlaceCone();
  void ArmDown();

  uint32_t AutoClock = 0;

  //cycles inlet mode
  int inMode = 0;

  typedef enum{Up_Pos, Down_Pos} ArmPosition;
  //declares where 4 bar arm is: 0 is at the the lower limit, 1 is in transit, 2 is at the upper limit.
  ArmPosition armPos = Down_Pos;
  //declares where carridge is: 0 is as far back as it can go, 1 is in transit, 2 is as far forward as it can go.
  int carPos = 1;

  static constexpr units::angle::turn_t TOP_ARM_POSITION {260E3};
  static constexpr units::angle::turn_t BOTTOM_ARM_POSITION {0};
  static constexpr units::angle::turn_t MIDDLE_ARM_POSITION {TOP_ARM_POSITION * 0.72};

  //PID for carriage
  static constexpr units::angle::turn_t FRONT_CARRIAGE_POSITION {-70000}; //inverting the motor should make it possible so you don't have to have negative forward movement
  static constexpr units::angle::turn_t BACK_CARRIAGE_POSITION {0};
  static constexpr units::angle::turn_t CENTERPOSITION = {(FRONT_CARRIAGE_POSITION - BACK_CARRIAGE_POSITION) / 2};

  //High 4 Bar Arm Limit switch
  frc::DigitalInput FourBarUpperLimitSwitch{0};
  //Low 4 Bar Arm limit switch
  frc::DigitalInput FourBarLowerLimitSwitch{1};
  //Front Carridge Limit Switch
  frc::DigitalInput FrontCarriageLS{2};
  //Rear Carridge Limit Switch
  frc::DigitalInput RearCarriageLS{3};

  Pneumatics _Pneumatics;
};

#endif