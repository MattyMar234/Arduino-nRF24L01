#ifndef JOYSTIC_H
#define JOYSTIC_H

#include "arduino.h"
#include "ToggleButton.h"

class Joystick : public ToggleButton
{
    private:
    
        uint8_t Pin_vrx;
        uint8_t Pin_vry;

    public:

        int16_t ValueX;
        int16_t ValueY;
        uint8_t ValueSW;

        Joystick(uint8_t Pin_sw, uint8_t Pin_vrx, uint8_t Pin_vry, uint8_t ModeSw);
        void ReadButtonState(boolean* s);
        boolean ReadButtonState();
        uint8_t getPinX();
        uint8_t getPinY();
        
};

#endif
