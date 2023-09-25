#include "Controls.h"

Controls::Controls() 
{
    uint8_t lenght = sizeof(joystickPin)/sizeof(joystickPin[0]);
    uint8_t *p = &joystickPin[0];

    this->joysticksNumber = lenght;
    this->joysticks = malloc(sizeof(Joystick)*lenght);
    this->analogReadState = lenght;



    for(uint8_t  i = 0; i < lenght; i++) {
        this->joysticksArray[i] = new Joystick(*(p+0), *(p+1), *(p+2), Bt_FALLING);
    
        p += sizeof(joystickPin[0])
    }

}

uint8_t Controls::getADC_MUX_address() 
{


    analogReadState = ++analogReadState %  joysticksNumber*2;
}


