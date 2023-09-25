#pragma once
#ifndef CONTROLS_H
#define CONTROLS_H

#include <arduino.h>
#include "Joystick.h"


static const uint8_t joystickPin[][] PROGMEM = {{46,A0,A1},{48,A2,A3}, {42,A4,A5}, {44,A6,A7}, {38,A8,A9}, {40,A10,A11}};


class Controls
{
    private:
        Joystick joysticksArray*;
        uint8_t joysticksNumber;
        uint8_t analogReadState;
        

        

    public:
        Controls();

};

#endif