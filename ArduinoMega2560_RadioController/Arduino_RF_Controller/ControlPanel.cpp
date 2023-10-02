#include "ControlPanel.h"

ControlPanel::ControlPanel() 
{
    this->ADC_cycle_Index = 0;
    this->joysticPointer = &joystickArray[0];

}

uint8_t ControlPanel::getCurrentJoystick_ADC_MUX_address() 
{
    if(ADC_cycle_Index % 2 == 0) 
        return this->joysticPointer->getPinX() - A0; 
    return this->joysticPointer->getPinY() - A0;
}

/**
 * @return return true se ho completato un ciclo
 */
boolean ControlPanel::nextJoystick() {
    ADC_cycle_Index = ++ADC_cycle_Index %  joysticksAnalogPinNumber;
    this->joysticPointer =  &joystickArray[(int)ADC_cycle_Index/2];

    return ADC_cycle_Index == 0 ? true : false;
}

void ControlPanel::setCurrentJoystick_PotenziometerValue(uint16_t value) {
    if(ADC_cycle_Index % 2 == 0) 
        this->joysticPointer->setValueX(value);
    else 
       this->joysticPointer->setValueY(value);
    /*    if(ADC_state % 2 == 0) 
      joysticks[index]->ValueX = (index % 2 == 1) ? adcVal : (1023 - adcVal);   //map(adcVal, 0, 1023, -512, +511) * ((ADC_state / 2) % 2 == 1) ? 1 : -1;
    else
      joysticks[index]->ValueY = (index % 2 == 1) ? (1023 - adcVal) : adcVal;  //map(adcVal, 0, 1023, -512, +511) * ((ADC_state / 2) % 2 == 1) ? -1 : 1;
    */
}


void ControlPanel::dumpControlsStatus() 
{
    static char buffer[40];

    for(uint8_t i = 0; i < this->joysticksNumber; i++) {
        sprintf(buffer, "J%01d [x:%04d,y:%04d,sw:%01d]|", i + 1, joystickArray[i].ValueX,  joystickArray[i].ValueY, joystickArray[i].ValueSW);
        DebugPort.print(buffer);
    }

    sprintf(buffer, "Pot1[v:%03d]");
    DebugPort.println(buffer);
}


