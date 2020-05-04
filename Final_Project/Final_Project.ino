#include "robot.h"

Robot robot;

void setup() {
  Serial.begin(115200);
  robot.Init();
}

void loop() {
  robot.executeStateMachine();
}
