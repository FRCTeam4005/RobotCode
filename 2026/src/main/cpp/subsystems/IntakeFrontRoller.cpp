#include "subsystems/IntakeFrontRoller.h"
#include "generated/TunerConstants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <frc/DoubleSolenoid.h>


#define INTAKE_IN frc::DoubleSolenoid::Value::kForward
#define INTAKE_OUT frc::DoubleSolenoid::Value::kReverse

IntakeFrontRoller::IntakeFrontRoller() : 
                            m_doubleSolenoid( CANConstants::kPneumaticHub,
                              frc::PneumaticsModuleType::REVPH,
                              PneumaticsChannelConst::kIntakeOutChannel,
                              PneumaticsChannelConst::kIntakeInChannel )
{
  
  
  IntakeFrontRollerMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CANConstants::kIntakeMotorID);
  IntakeFrontRollerMotor->Set(0);
  RollerIn();
  SetName("IntakeFrontRoller");
}



frc2::CommandPtr IntakeFrontRoller::Out()
{
  return frc2::FunctionalCommand(
      [this] {},
      [this] {RollerOut(); setSpeed(.5);},
      [this] (bool interrupted) {RollerIn(); setSpeed(0);},
      [this] {return false;},
      {this}
  ).ToPtr();
}

frc2::CommandPtr IntakeFrontRoller::Unstick()
{
  return frc2::FunctionalCommand(
    [this] {},
    [this] {RollerOut(); setSpeed(-0.5);},
    [this] (bool interrupted) {RollerOut(); setSpeed(0);},
    [this] {return false;},
    {this}
  ).ToPtr();
}

frc2::CommandPtr IntakeFrontRoller::In()
{
  return frc2::FunctionalCommand(
    [this] {},
    [this] {RollerIn(); setSpeed(0);},
    [this] (bool interrupted) {},
    [this] {return true;},
    {this}
  ).ToPtr();
}

frc2::CommandPtr IntakeFrontRoller::StopIntake()
{
  return frc2::FunctionalCommand(
    [this] {},
    [this] {setSpeed(0);},
    [this] (bool interrupted) {},
    [this] {return true;},
    {this}
  ).ToPtr();
}

frc2::CommandPtr IntakeFrontRoller::Momentary()
{
  return frc2::FunctionalCommand(
    [this] {},
    [this] {RollerOut(); setSpeed(-1);},
    [this] (bool interrupted) {RollerIn(); setSpeed(.5);},
    [this] {return false;},
    {this}
  ).ToPtr();
}

void IntakeFrontRoller::setSpeed(double speed)
{
  IntakeFrontRollerMotor->Set(-speed);
}
 
void IntakeFrontRoller::RollerJog(double speed)
{
  setSpeed(speed);
}
 
void IntakeFrontRoller::RollerIn()
{
  m_doubleSolenoid.Set(INTAKE_IN);
}
 
void IntakeFrontRoller::RollerOut()
{
  m_doubleSolenoid.Set(INTAKE_OUT);
}
