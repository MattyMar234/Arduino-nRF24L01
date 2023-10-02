#include "ToggleButton.h"


ToggleButton::ToggleButton(uint8_t pin, uint8_t mode, unsigned long debounce_time) 
{

    this->PIN = pin;
    this->Mode = mode;
    this->debounce_time = debounce_time;
    last_debounce_time = millis() - debounce_time;
    
    pinMode(pin, INPUT);
    last_state   = utils::fastDigitalRead(pin);
    button_state = utils::fastDigitalRead(pin);
}

bool ToggleButton::getState() {
    return utils::fastDigitalRead(this->PIN);
}

uint8_t ToggleButton::getPin() {
    return this->PIN;
}

void ToggleButton::setPin(uint8_t pin) {
    this->PIN = pin;
}

void ToggleButton::setDebounce_time(unsigned long debounce_time) {
    this->debounce_time = debounce_time;
}

unsigned long ToggleButton::getDebounce_time() {
    return this->debounce_time;
}

bool ToggleButton::isButtonFunctionAvailable()
{
    //se è passato il tempo eseguo
    if((millis() - last_debounce_time) > debounce_time)
    {
        bool result = false;
        this->button_state = utils::fastDigitalRead(this->PIN);

        switch (this->Mode)
        {
            //change
            case BT_CHANGE:
                if(this->button_state != this->last_state) {
                    this->last_state = this->button_state;
                    result = true;
                }
                break;
            
            //falling
            case BT_FALLING:
                if(this->button_state != this->last_state) {
                    this->last_state = this->button_state;

                    if(button_state == LOW) {
                        result = true;
                    }     
                }
                break;

            //rising
            case BT_RISING:
                if(this->button_state != this->last_state) {
                    this->last_state = this->button_state;

                    if(button_state == HIGH) {
                        result = true;
                    }     
                }
                break;
        }
        
        if(result) {
            last_debounce_time = millis();
        } 
        return result;
    } 
    else {
        return false;
    }
}
