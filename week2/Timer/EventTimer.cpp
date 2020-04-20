# include "EventTimer.h"

EventTimer::EventTimer() {
}

void EventTimer::Start (uint32_t interval) {
  if (state == 0) {
    timeInterval = interval;
    startTime = millis(); // captures elapsed time
    timerExpired = false; // sets flag to false
    state = 1; // updates state
  }
}

bool EventTimer::CheckExpired (void) {

  if (state == 1 ) {
    if (millis() - startTime >= timeInterval) { // checks if timer expired
      state = 2; // update the state
      timerExpired = true; // sets flag to true
      return true;
    }
    else return false;
  }
  else return false;
}

void EventTimer::Cancel (void) {
  if (state == 2) {
    startTime = 0; // resets start time to 0
    state = 0; // resets state to 0
  }
}
