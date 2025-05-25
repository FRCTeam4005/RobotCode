#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <thread>
#include <chrono>
#include <memory>
#include <mutex>
#include "subsystems/LimeLightData.h"

#define LED_ON      3u
#define LED_BLINK   2u
#define LED_OFF     1u

class LimeLight {

public:
    LimeLight(LimeLightData* limelightdata, std::string tableName = "limelight");    
    void setLedOn();
    void setLedOff();
    void setLedBlink();

private:
    LimeLightData* LimeLightData_;
    std::unique_ptr<std::thread> cameraServer;

    std::mutex networkTableMtx;
    std::shared_ptr<nt::NetworkTable> table;

    int updateCameraData();
};
