#pragma once
#ifndef CONTROLS_H
#define CONTROLS_H

#define POTENZIOMETER_PIN A12

#include <arduino.h>
#include "Joystick.h"
#include "globalVariable.h"


class ControlPanel
{
    private:
        Joystick joystickArray[6] = {
            Joystick(46, A0,  A1,   BT_FALLING, false, true),    //J1
            Joystick(48, A2,  A3,   BT_FALLING, true, false),    //J2
            Joystick(42, A4,  A5,   BT_FALLING, false, true),    //J3
            Joystick(44, A6,  A7,   BT_FALLING, true, false),    //J4
            Joystick(38, A8,  A9,   BT_FALLING, false, true),    //J5
            Joystick(40, A10, A11,  BT_FALLING, true, false),    //J6
        };

        uint8_t joysticksNumber = sizeof(joystickArray)/sizeof(joystickArray[0]);
        uint8_t joysticksAnalogPinNumber = joysticksNumber*2;
        uint8_t ADC_cycle_Index;
        Joystick* joysticPointer;
        uint16_t potenziometreValue;
        
    protected:
        

    public:
        ControlPanel();
        
        boolean init() {return true;}
        void dumpControlsStatus();
        uint8_t getCurrentJoystick_ADC_MUX_address();
        boolean nextJoystick();
        void setCurrentJoystick_PotenziometerValue(uint16_t value);
        
        uint8_t getPotenziometre_ADC_MUX_address() {
            return POTENZIOMETER_PIN - A0;
        }

        void setPotenziometreValue(uint16_t value) {
            potenziometreValue = value;
        }

        uint8_t getNumberOfJoysticks() {
            return joysticksNumber;
        }

        Joystick* getJoystick(uint8_t index) {
            if(index >= joysticksNumber)
                return nullptr;
            return &joystickArray[index];
        }


        
    

};


#endif