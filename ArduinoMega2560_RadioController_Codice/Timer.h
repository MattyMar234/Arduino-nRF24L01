#ifndef TIMER_h
#define TIMER_h

#include <arduino.h>

class Timer 
{
  private:

      unsigned long previousMillis;
      unsigned long currentMillis;
      unsigned long interval;

  public:

      Timer(unsigned long Interval); //in millis
      void setInterval(unsigned long Interval);
      void Update();
      bool Test();

};

/*

struct Timer {
  unsigned long previousMillis;
  unsigned long currentMillis;
  unsigned long interval;
};

extern boolean TestTimer(struct Timer *TIMER){
  TIMER->currentMillis = millis();

  if (TIMER->currentMillis - TIMER->previousMillis >= TIMER->interval){
    TIMER->previousMillis = TIMER->currentMillis;
    return true;
  }
  else{
    return false;
  }
}*/

#endif
