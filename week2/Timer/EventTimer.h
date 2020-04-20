# include <Arduino.h>
#include <stdint.h>

#ifndef __EVENTTIMER_H
#define __EVENTTIMER_H

class EventTimer {
  private:
    uint32_t timeInterval; // Captures the interval parameter of the Start method
    uint8_t state = 0; // Keep strack of the state to execute the methods
    uint32_t startTime; // to capute time in start method
    bool timerExpired = false; // flag to check if timer expired

  public:
    EventTimer (); 
    void Start (uint32_t interval);
    bool CheckExpired(void);
    void Cancel(void);
};

# endif
