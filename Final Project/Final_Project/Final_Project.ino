#include "robot.h"

Robot robo;

void setup() {
  Serial.begin(115200);
  robo.Init();
}

void loop() {
  robo.executeStateMachine();
}
