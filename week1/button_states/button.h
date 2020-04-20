# include "Arduino.h"
#include <stdint.h>

#ifndef __BUTTON_H
#define __BUTTON_H

class Button 
{
  private :
    enum BUTTON_STATE {BUTTON_STABLE, BUTTON_UNSTABLE};
    BUTTON_STATE state = BUTTON_STABLE;

    uint8_t buttonPin = -1;

    //assume active Low
    uint8_t buttonPosition = HIGH; //most recent stable position
    uint8_t tempButtonPos = HIGH; //temporary position, bouncing not complete

    uint32_t lastBounceTime = 0;
    uint32_t debouncePeriod = 10; // in ms

  public:
    Button(uint8_t pin, uint32_t db=10); //default to 10 ms debounce
    void Init(bool usePullup = true);
    bool CheckButtonPress(void);
};

#endif
