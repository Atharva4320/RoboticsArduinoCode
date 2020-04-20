
# include "EventTimer.h"

EventTimer myTimer; // EventTimer object

void setup() {
  Serial.begin (115200);
  while (!Serial) {} //you must open the Serial Monitor to get past this step!
  Serial.println("Start Timer!");
}

unsigned long timerCount = 0;


void loop() {

  myTimer.Start(1000); // set timer of 1000 ms -> 1s
  if (myTimer.CheckExpired()) {

    Serial.print("Expired Timer counts: "); // prints out the number of times the timer has expired
    Serial.print('\t');
    Serial.print(++timerCount);
    Serial.print('\n');

    myTimer.Cancel();
  }
}
