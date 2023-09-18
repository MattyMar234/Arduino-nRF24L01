#include "Timer.h"

Timer::Timer(unsigned long Interval) {
    this->interval = Interval;
    this->currentMillis = 0;
    this->previousMillis = 0;
}


void Timer::setInterval(unsigned long Interval) {
    this->interval = Interval;
}


void Timer::Update() {
    currentMillis = millis();
}


bool Timer::Test() {
    currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    return true;
  }
  else{
    return false;
  }
}