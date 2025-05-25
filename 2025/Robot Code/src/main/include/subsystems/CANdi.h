#pragma once

#include <ctre/phoenix6/CANdi.hpp>
#include <atomic>

class CANDigitalInput {
public:
    CANDigitalInput(uint8_t CANID);

    auto IsClawBreakBeamActive() -> bool;
    auto IsElevatorMagSensorActive() -> bool;

    auto ScanInputs() -> void;

private:
    std::unique_ptr<ctre::phoenix6::hardware::CANdi> candi;
    std::atomic_bool IsClawBBActive;
    std::atomic_bool IsElevatorMagActive;
};