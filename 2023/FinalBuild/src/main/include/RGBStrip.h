#if !defined(RGB_STRIP_H)
#define RGB_STRIP_H
#include <frc/DigitalOutput.h>
#include "DriverController.h"

class RGBStrip
{
private:

    typedef enum
    {
        YELLOW,
        PURPLE,
    }RGB_COLORS;

    bool currColor = YELLOW;
    frc::DigitalOutput _0thBit{9};
public:
    RGBStrip()
    {}

    void RGB_Color_Toggle(DriverController &Controller)
    {
        if(Controller.Get_RGB_Toggle_Button())
        {
            currColor = !currColor;
        }

        _0thBit.Set(currColor);
    }
};
#endif



