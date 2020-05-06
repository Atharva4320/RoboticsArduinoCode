#include <filter.h>
#include <RemoteDecoder.h>
#include <RemoteConstants.h>
#include "robot.h"
#include <params.h> // PID library
#include <serial_comm.h> // PID library

// defining ultrasonic sensor states for pulse capture:
enum PULSE_STATE {PLS_IDLE, PLS_WAITING_LOW, PLS_WAITING_HIGH, PLS_CAPTURED};
PULSE_STATE pulseState; // initialized to IDLE

volatile uint16_t pulseStart = 0;
volatile uint16_t pulseEnd = 0;
float targetDistance = 30.0;
float actualDistance = 0;
float targetSpeed = 0;
float baseLeft = 10.0;
float baseRight = 10.0;
const uint8_t trigPin = 14; //this may be any appropriate pin, connect the pin to Trig on the sensor
uint32_t lastPing = 0; //for scheduling pings
uint32_t PING_INTERVAL = 250; //ms
volatile uint8_t readyToPID = 0;   //a flag that is set when the PID timer overflows
uint16_t lineSensorValues[NUM_SENSORS];
bool useEmitters = true; //  to use the emmiters of the line sensors
bool lineNotDetected = true; // flag to check if the line has been reached
bool lineDetected = false;
bool lineEnded = false;
bool irDetected = false;
bool turned = false;
bool onRamp = false;
bool rampComplete = false;

volatile int16_t countsLeft = 0;
volatile int16_t countsRight = 0;

// Remote Control:
bool messageActive = false;
uint16_t lastMessageTimeMs = 0;
const uint16_t messageTimeoutMs = 115;

// Sensor Fusion:
float estimatedAngle;
float accOffset;
float gyroBias;
float observedAngle;
float est;

Button buttonC (17); //button C is pin 17 on the Zumo
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
LSM303 compass;
L3G gyro;

ComplementaryFilter filter (compass, gyro, estimatedAngle, accOffset, gyroBias);


RemoteDecoder decoder;

Robot::Robot() {}

void Robot::Init() {
  Serial.begin(115200);
  //while (!Serial) {} //you must open the Serial Monitor to get past this step!
  Serial.println("Hi!");

  noInterrupts(); //disable interupts while we mess with the control registers

  //sets timer 3 to normal mode (16-bit, fast counter)
  TCCR3A = 0;

  //sets up timer 4
  TCCR4A = 0x00; //disable some functionality -- no need to worry about this
  TCCR4B = 0x0C; //sets the prescaler -- look in the handout for values
  TCCR4C = 0x04; //toggles pin 6 at one-half the timer frequency
  TCCR4D = 0x00; //normal mode

  OCR4C = 124;   //TOP goes in OCR4C
  TIMSK4 = 0x04; //enable overflow interrupt

  interrupts(); //re-enable interrupts

  buttonC.Init(); //don't forget to call Init()!

  pinMode(trigPin, OUTPUT);
  pinMode(13, INPUT); //explicitly make 13 an input, since it defaults to OUTPUT in Arduino World (LED)

  lastPing = millis();
  lineSensors.initFiveSensors(); // initialized 5 line sensors
  //state = ROBOT_IDLE;
  Serial.println("Initialized state as IDLE");
  //pulseState = PLS_IDLE;

  decoder.init();

  filter.Init();
}

void Robot::CommandPing(int trigPin) {
  cli(); //disable interrupts

  TIFR3 = 0x20; //clear any interrupt flag that might be there

  TIMSK3 |= 0x20; //enable the input capture interrupt
  TCCR3B |= 0xC0; //set to capture the rising edge on pin 13; enable noise cancel

  sei(); //re-enable interrupts

  //update the state and command a ping
  pulseState = PLS_WAITING_LOW;

  digitalWrite(trigPin, HIGH); //command a ping by bringing TRIG HIGH
  delayMicroseconds(10);      //we'll allow a delay here for convenience; it's only 10 us
  digitalWrite(trigPin, LOW);  //must bring the TRIG pin back LOW to get it to send a ping
}

void Robot::distancePID() {
  //schedule pings roughly every PING_INTERVAL milliseconds
  if (millis() - lastPing > PING_INTERVAL)
  {
    lastPing = millis();
    CommandPing(trigPin); //command a ping
  }

  if (pulseState == PLS_CAPTURED) //we got an echo
  {
    //update the state to IDLE
    pulseState = PLS_IDLE;

    /*
       Calculate the length of the pulse (in timer counts!). Note that we turn off
       interrupts for a VERY short period so that there is no risk of the ISR changing
       pulseEnd or pulseStart. As noted in class, the way the state machine works, this
       isn't a problem, but best practice is to ensure that no side effects can occur.
    */
    noInterrupts();
    uint16_t pulseLengthTimerCounts = pulseEnd - pulseStart;
    interrupts();

    uint32_t pulseLengthUS = pulseLengthTimerCounts * 4; //pulse length in us

    actualDistance = (pulseLengthUS - 84.519) / 57.252; //distance in cm

  }


  float  constrainedDistance = constrain(actualDistance, 0, targetDistance + 10);
  noInterrupts();
  float distanceError =  targetDistance - constrainedDistance ;
  static float prevDistError = 0;

  static float distanceSum = 0;
  distanceSum += distanceError;

  static float distanceDiff = 0;
  distanceDiff = distanceError - prevDistError;

  targetSpeed =  0.5 * distanceError + 25 * distanceDiff;

  prevDistError = distanceError;
  interrupts();

}

void Robot::linePID() {
  float leftSensor = lineSensorValues[1];
  float centerSensor = lineSensorValues[2];
  noInterrupts();
  float lineError = (leftSensor - centerSensor) / 10; // to make the error more manageable

  //  if (lineNotDetected == false) {
  //    Serial.println(lineError);
  //  }
  static float prevLineError = 0;

  static float lineDiff = 0;
  lineDiff = lineError - prevLineError;

  targetSpeed = /*4 * lineDiff;// */ 3.2 * lineError ;//+ 25 * lineDiff;
  if (lineNotDetected == false) {
    //    Serial.print(lineError);
    //    Serial.print('\t');
    //    Serial.print(lineDiff);
    //    Serial.print('\t');
    //    Serial.print(targetSpeed);
    //    Serial.print('\n');
  }
  interrupts();
}

bool Robot::lineFinished() {
  if (state == ROBOT_LINE_FOLLOW) {
    if (lineSensorValues[2] > 300) {
      lineEnded = true;
    }
    else lineEnded = false;
  }
  return lineEnded;
}

void Robot::motorPID() {

  //clear the timer flag
  readyToPID = 0;

  float targetLeft = baseLeft + targetSpeed;
  float targetRight = baseRight - targetSpeed;

  //for tracking previous counts
  static int16_t prevLeft = 0;
  static int16_t prevRight = 0;

  //error sum
  static int16_t sumLeft = 0;
  static int16_t sumRight = 0;


  /*
     Do PID stuffs here. Note that we turn off interupts while we read countsLeft/Right
     so that it won't get accidentally updated (in the ISR) while we're reading it.
  */
  noInterrupts();
  int16_t speedLeft = countsLeft - prevLeft;
  int16_t speedRight = countsRight - prevRight;

  prevLeft = countsLeft;
  prevRight = countsRight;
  interrupts();
  // use Kd here:


  int16_t errorLeft = targetLeft - speedLeft;
  sumLeft += errorLeft;
  //diffLeft -= errorLeft;

  int16_t errorRight = targetRight - speedRight;
  sumRight += errorRight;
  //diffRight -= errorRight;

  float effortLeft = 10 * errorLeft + 0.2 * sumLeft; // + Kd * diffLeft;
  float effortRight = 10 * errorRight + 0.2 * sumRight; // + Kd * diffRight;

  motors.setSpeeds(effortLeft, effortRight); //up to you to add the right motor

  if (CheckSerialInput()) {
    ParseSerialInput();
  }

  /* for reading in gain settings
     CheckSerialInput() returns true when it gets a complete string, which is
     denoted by a newline character ('\n'). Be sure to set your Serial Monitor to
     append a newline
  */
  if (lineNotDetected == false) {
    //    Serial.print(targetSpeed);
    //    Serial.print('\t');
    //    Serial.print(targetLeft);
    //    Serial.print('\t');
    //    Serial.print(targetRight);
    //    Serial.print('\n');
   // Serial.println(lineEnded);

  }
}

bool Robot::detectLine() {
  // Read the line sensors.
  lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
  if (state == ROBOT_WALL_FOLLOW) {
    if (lineSensorValues[0] < 150 || lineSensorValues[2] < 150 || lineSensorValues[4] < 150) {
      lineDetected = true;
    }
    else lineDetected = false;
    return lineDetected;
  }
}

bool Robot::detectIR() {
  //  if (state == ROBOT_LINE_FOLLOW) {
  //
  //    decoder.service();
  //
  //    // Turn on the yellow LED if a message is active.
  //    ledYellow(messageActive);
  //
  //    // Turn on the red LED if we are in the middle of receiving a
  //    // new message from the remote.  You should see the red LED
  //    // blinking about 9 times per second while you hold a remote
  //    // button down.
  //    ledRed(decoder.criticalTime());
  //
  //    if (decoder.criticalTime())
  //    {
  //      // We are in the middle of receiving a message from the
  //      // remote, so we should avoid doing anything that might take
  //      // more than a few tens of microseconds, and call
  //      // decoder.service() as often as possible.
  //    }
  //    else
  //    {
  ////      if (decoder.getAndResetMessageFlag())
  ////      {
  ////        // The remote decoder received a new message, so record what
  ////        // time it was received and process it.
  ////        lastMessageTimeMs = millis();
  ////        messageActive = true;
  ////      }
  ////
  ////      if (decoder.getAndResetRepeatFlag())
  ////      {
  ////        // The remote decoder receiver a "repeat" command, which is
  ////        // sent about every 109 ms while the button is being held
  ////        // down.  It contains no data.  We record what time the
  ////        // repeat command was received so we can know that the
  ////        // current message is still active.
  ////        lastMessageTimeMs = millis();
  ////      }
  //      irDetected = true;
  //    }
  //    // Check how long ago the current message was last verified.
  //    // If it is longer than the timeout time, then the message has
  //    // expired and we should stop executing it.
  //
  //    if (messageActive && (uint16_t)(millis() - lastMessageTimeMs) > messageTimeoutMs)
  //    {
  //      messageActive = false;
  //    }
  //    return irDetected;
  //  }
}

bool Robot::detectHorizon() {

}

void Robot::rampAngle() {
  Serial.println("Over here");
  //if (state == ROBOT_RAMP) {
    if (filter.calcAngle(observedAngle,est,gyroBias)) {
      if (est > 0.6) {
        onRamp = true;
        rampComplete = false;
      }
      if ((est < 0.3) && (onRamp == true)) {
        rampComplete = true;
        onRamp = false; 
      }
    }
//  }
}

void Robot::turnTillLine() {
  Serial.println("In turnTillLine() function");
  motors.setSpeeds(200, -200);
  if (state == ROBOT_LINE_FOLLOW) {
    if (lineSensorValues[2] < 160) {
      //state = ROBOT_IDLE;
      timer.Start(8000);
      motors.setSpeeds(0, 0);
      lineNotDetected = false;
      lineEnded = false;
    }
    else lineNotDetected = true;
    return lineNotDetected;
  }
}

void Robot::HandleButtonPress() {
  if (state == ROBOT_IDLE) {
    Serial.println("In HandleButtonPress()");
    timer.Start(1000);
    Serial.println("timer started");
    state = ROBOT_WAITING;
    Serial.println("state changed");
  }
}

void Robot::HandleTimerExpired () {
  Serial.println ("In HandleTimerExpired() ");
  if (state == ROBOT_WAITING) {
    Serial.println("Executed when state in ROBOT_WAITING");
    state = ROBOT_WALL_FOLLOW;
    Serial.println("State changed");
    timer.Cancel();
    Serial.println("Timer cancelled");
  }
  else if (state == ROBOT_WALL_FOLLOW) {
    Serial.println("Executed when state in ROBOT_LINE_FOLLOW");
    timer.Cancel();
    Serial.println("Timer cancelled");
    Serial.println("Gets executed till here....");
    Serial.println("Call to turnTillLine() function");
    //timer.Start(1000);
    // set motor Speeds to 0
    state = ROBOT_LINE_FOLLOW;
  }
  else if (state == ROBOT_LINE_FOLLOW) {
    timer.Cancel();
    // Code gets substituted to HandleIrDetected()
    timer.Start(550);
    motors.setSpeeds(150, -150);
    state = ROBOT_RAMP;
  }
  else if (state == ROBOT_RAMP) {
    timer.Cancel();
    turned = true;
  }

  //  else if (state == ROBOT_RAMP) {
  //    state = ROBOT_RAMP;
  //    Serial.println("Executed when state in ROBOT_RAMP");
  //    timer.Cancel();
  //    Serial.println("Timer cancelled");
  //    motors.setSpeeds(200, 200);
  //    //write a function to check for sensor fusion reading -> call to detectHorizon()
  //    // Serial.print ("Sets speed back to 0");
  //  }
}
void Robot::turn() {

}

void Robot::HandleLineDetected() {
  if (state == ROBOT_WALL_FOLLOW) {
    timer.Start(150);
    motors.setSpeeds(100, 100);
  }
}


void Robot::HandleIrDetected() {
  if (state == ROBOT_LINE_FOLLOW) {
    state = ROBOT_IDLE;
  }
}

void Robot::HandleHorizonDetected() {

}


void Robot::executeStateMachine() {

  // if statements to execute the state machine:
  if (buttonC.CheckButtonPress()) HandleButtonPress();
  if (timer.CheckExpired()) HandleTimerExpired();
  if (detectLine()) HandleLineDetected();
  if (detectIR()) HandleIrDetected();
  if (detectHorizon()) HandleHorizonDetected();

  // actual state machine:
  switch (state) {
    case ROBOT_IDLE:
      motors.setSpeeds(0, 0);
      break;
    case ROBOT_WAITING:
      motors.setSpeeds(0, 0);
      break;
    case ROBOT_WALL_FOLLOW:
      if (readyToPID) {
        distancePID();
        motorPID();
      }
      break;
    case ROBOT_LINE_FOLLOW:
      if (lineNotDetected) {
        turnTillLine();
      }
      if (readyToPID) {
        linePID();
        motorPID();
        detectIR();
        lineFinished();
      }
      if (lineEnded) {
        motors.setSpeeds(0, 0);
      }
      break;
    case ROBOT_RAMP:
      //add code
      Serial.print(onRamp);
      Serial.print('\t');
      Serial.print(rampComplete);
      Serial.print('\t');
      Serial.print(est);
      Serial.print('\n');
      if (turned) {
        motors.setSpeeds(300, 265);
        rampAngle();
      }
      if (rampComplete) {
        Serial.println("Ramp Completed");
        motors.setSpeeds(0,0);
        state = ROBOT_IDLE;
      }
      break;
    case ROBOT_360_TURN:
      //add code
      break;
  }
}

/*
   ISR for input capture on pin 13. We can precisely capture the value of TIMER3
   by setting TCCR3B to capture either a rising or falling edge. This ISR
   then reads the captured value (stored in ICR3) and copies it to the appropriate
   variable.
*/

// TIMERS:-

//Don't know why I'm getting a 'pulseState' was not declared error
ISR(TIMER3_CAPT_vect) {
  if (pulseState == PLS_WAITING_LOW) //we're waiting for a rising edge
  {
    pulseStart = ICR3; //copy the input capture register (timer count)
    TCCR3B &= 0xBF;    //now set to capture falling edge on pin 13
    pulseState = PLS_WAITING_HIGH;
  }

  else if (pulseState == PLS_WAITING_HIGH) //waiting for the falling edge
  {
    pulseEnd = ICR3;
    pulseState = PLS_CAPTURED; //raise a flag to indicate that we have data
  }
}

ISR(TIMER4_OVF_vect) {
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();

  readyToPID = 1;
}
