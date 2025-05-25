#ifndef LIMELIGHTVISION_H
#define LIMELIGHTVISION_H

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <units/angle.h>
#include <units/length.h>

#define LED_ON      3u
#define LED_BLINK   2u
#define LED_OFF     1u

struct CameraDistance 
{
    units::inch_t forward;
    units::inch_t sideways;
    int Tilda_ID;
};

class LimeLightVision {

public:

    LimeLightVision();    
    int getTargetDistance(CameraDistance& cameradistance);
    bool if_valid_target();
    void setLedOn();
    void setLedOff();
    void setLedBlink();
    void setStream(double option);
    double getStream();

private:
    units::inch_t cameraHeight{49.0};
    units::inch_t targetHeight{14.25};//this could be wrong adding 4ish inches mights help
    units::degree_t cameraAngle{0};
    std::shared_ptr<nt::NetworkTable> table;
    bool tv;
    double tx;
    double ty;
    double ta;
    double ts;
    double tl;
    int tildaID;
    nt::NetworkTableEntry stream;
};

class CameraAngles {
    public:
        CameraAngles(double tx, double ty);
        double getTx();
        double getTy();
    
    private:
        double tx;
        double ty;

};


#endif