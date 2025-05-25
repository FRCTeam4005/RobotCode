#include <subsystems/CANdi.h>

//break beam active when the beam is broken RX and TX can't "see" each other
#define CLAW_BREAKBEAM_ACTIVE_STATE ctre::phoenix6::signals::S2StateValue::Low
//break beam idle when the beam is not broken RX and TX can "see" each other
#define CLAW_BREAKBEAM_IDLE_STATE ctre::phoenix6::signals::S2StateValue::High

//magnet active is 0 when the magnet is close to the hall effect sensor
#define ELEVATOR_MAG_ACTIVE_STATE ctre::phoenix6::signals::S1StateValue::Low
//magnet idle is 1 (5V) when the magnet is not close to the hall effect sensor
#define ELEVATOR_MAG_IDLE_STATE ctre::phoenix6::signals::S1StateValue::High

CANDigitalInput::CANDigitalInput(uint8_t CANID)
{
    candi = std::make_unique<ctre::phoenix6::hardware::CANdi>(CANID);
}

bool  CANDigitalInput::IsClawBreakBeamActive()
{
    return IsClawBBActive.load();
}

bool CANDigitalInput::IsElevatorMagSensorActive()
{
    return IsElevatorMagActive.load();
}

void CANDigitalInput::ScanInputs()
{
    IsElevatorMagActive.store(candi->GetS1State().GetValue() == ELEVATOR_MAG_ACTIVE_STATE);
    IsClawBBActive.store(candi->GetS2State().GetValue() == CLAW_BREAKBEAM_ACTIVE_STATE);
}