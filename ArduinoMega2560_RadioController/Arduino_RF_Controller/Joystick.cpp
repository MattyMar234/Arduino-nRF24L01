#include "ToggleButton.h"
#include "Joystick.h"


Joystick::Joystick(uint8_t Pin_sw, uint8_t Pin_vrx, uint8_t Pin_vry, uint8_t ModeSw, boolean reverseX, boolean reverseY) : ToggleButton(Pin_sw, ModeSw, 100)
{
    this->Pin_vrx = Pin_vrx;
    this->Pin_vry = Pin_vry;
    this->reverseX = reverseX;
    this->reverseY = reverseY;

    pinMode(Pin_vrx,INPUT);
    pinMode(Pin_vry,INPUT);
    
}



void Joystick::ReadButtonState(boolean* s) {
  *s = isButtonFunctionAvailable();
}

boolean Joystick::ReadButtonState() {
  return isButtonFunctionAvailable();
}

uint8_t Joystick::getPinX(){ return this->Pin_vrx; }
uint8_t Joystick::getPinY(){ return this->Pin_vry; }
