#ifndef JOYSTIC_H
#define JOYSTIC_H

#include "arduino.h"
#include "ToggleButton.h"


class Joystick : public ToggleButton
{
    private:
        uint8_t Pin_vrx;
        uint8_t Pin_vry;
        boolean reverseX;
        boolean reverseY;

    public:

        int16_t ValueX;
        int16_t ValueY;
        uint8_t ValueSW;

        Joystick(uint8_t Pin_sw, uint8_t Pin_vrx, uint8_t Pin_vry, uint8_t ModeSw, boolean reverseX, boolean reverseY);
        void ReadButtonState(boolean* s);
        boolean ReadButtonState();
        uint8_t getPinX();
        uint8_t getPinY();

        void setValueX(int16_t x) {
            ValueX = (reverseX) ? 1023 - x : x;
        }

        void setValueY(int16_t y) {
            ValueY = (reverseY) ? 1023 - y : y;
        }
        
};

#endif
