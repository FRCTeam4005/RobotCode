#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <frc/DriverStation.h>

struct LimeLightData 
{
    explicit LimeLightData(std::optional<frc::DriverStation::Alliance>);
    std::atomic<float> forward;
    std::atomic<float> sideways;
    std::atomic<float> X;
    std::atomic<float> Y;
    std::atomic<float> Z;
    std::atomic<float> Yaw;
    std::atomic<float> Pitch;
    std::atomic<float> Roll;
    std::atomic<int> Tilda_ID;
    std::atomic<float> TargetArea;

    //TODO: Make this a command step in auto?
    auto Set_botPoseSide(std::optional<frc::DriverStation::Alliance>) -> void;
    auto Get_botPoseSide() -> std::string;

    auto Set_botPoseData(nt::NetworkTable*) -> void;
    auto Get_botPoseVect() -> std::vector<double>;

private:
    std::mutex botPoseMtx;
    std::vector<double> botPose;

    std::atomic<unsigned int> Pipline = 0; //TODO ask Patrick if this is necsssary or even proper
    std::string botPoseSide;

    auto SetPoseSide(std::optional<frc::DriverStation::Alliance>) -> void;
};
