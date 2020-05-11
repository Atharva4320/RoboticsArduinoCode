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
const uint8_t trigPin = 14; //this may be any appropriate pin, connect the pin to Trig on the sensor
uint32_t lastPing = 0; //for scheduling pings
uint32_t PING_INTERVAL = 250; //ms
volatile uint8_t readyToPID = 0;   //a flag that is set when the PID timer overflows

static int16_t prevLeft = 0;
static int16_t prevRight = 0;

volatile int16_t countsLeft = 0;
volatile int16_t countsRight = 0;

// Sensor Fusion variables:
float estimatedAngle;
float accOffset;
float gyroBias;
float observedAngle;
float est;

// Creating objects:
EventTimer timer;
Button buttonC (17); //button C is pin 17 on the Zumo
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
LSM303 compass;
L3G gyro;

ComplementaryFilter filter (compass, gyro, estimatedAngle, accOffset, gyroBias);


RemoteDecoder decoder;

Robot::Robot() {}

/*
   Function Init() initializes all the variables to establish communication with the robot
*/
void Robot::Init() {
  Serial.begin(115200);

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

  buttonC.Init(); //initializing the button class

  pinMode(trigPin, OUTPUT);
  pinMode(13, INPUT); //explicitly make 13 an input, since it defaults to OUTPUT in Arduino World (LED)

  lastPing = millis();
  lineSensors.initThreeSensors(); // initialized 3 line sensors
  Serial.println("Initialized state as IDLE");

  decoder.init(); // initialized the decoder for IR detection

  filter.Init(); // initalized the filter for ramp and 360 turn detection
}

/*
   Funciton CommandPing(int trigPin) sends out an ultrasonic ping
*/
void Robot::CommandPing(int trigPin) {
  cli(); //disable interrupts

  TIFR3 = 0x20; //clear any interrupt flag that might be there

  TIMSK3 |= 0x20; //enable the input capture interrupt
  TCCR3B |= 0xC0; //set to capture the rising edge on pin 13; enable noise cancel

  sei(); //re-enable interrupts

  //update the state and command a ping
  pulseState = PLS_WAITING_LOW;

  digitalWrite(trigPin, HIGH); //command a ping by bringing TRIG HIGH
  delayMicroseconds(10);      //10 microseconds delay
  digitalWrite(trigPin, LOW);  //must bring the TRIG pin back LOW to get it to send a ping
}

/*
   Function distancePID() calculates how far or close the robot is from the target distance and sets the
   target speed accordingly using PID algorithm. Used for Wall Following
*/
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

    uint32_t pulseLengthUS = pulseLengthTimerCounts * 4; //calculating pulse length in us

    actualDistance = (pulseLengthUS - 84.519) / 57.252; //calculating distance in cm

  }

  noInterrupts();
  float distanceError =  targetDistance - actualDistance; //calculating the distance error
  static float prevDistError = 0; //initializing previous distance error to 0

  static float distanceDiff = 0; //initializing distance difference to 0
  distanceDiff = distanceError - prevDistError; //xalculating the distance difference for derivative control

  targetSpeed =  0.25 * distanceError + 50 * distanceDiff; //setting the target speed using PID algorithm with Kp as 0.25 and Kd as 50

  prevDistError = distanceError; // updating the previous distance error
  interrupts();

}

/*
   Function linePID()locates where the line is using line sensors and sets the target speed accordingly using PID algorithm.
   Used for Line Following
*/
void Robot::linePID() {
  float leftSensor = lineSensorValues[1];
  float centerSensor = lineSensorValues[2];

  noInterrupts();
  float lineError = ((2.8 * leftSensor) - centerSensor); //to make the error more manageable

  targetSpeed =  0.1 * lineError; //calculating the target speed using PID algorithm with Kp as 0.1
  interrupts();
}

/*
   Function lineFinished() checks if the line is ended
*/
bool Robot::lineFinished() {
  if (state == ROBOT_LINE_FOLLOW) { //function to be executed only if in LINE_FOLLOW state
    if (lineSensorValues[1] > 300) { //threshold value to check for the line
      lineEnded = true;
    }
    else lineEnded = false;
  }
  return lineEnded;
}

/*
   Function motorPID() calculates the effort required for each motor to reach the net target speed
   The target speed calculated from linePID() and distancePID() is used to set the final motor speed using PID algorithm
*/
void Robot::motorPID() {

  //clear the timer flag
  readyToPID = 0;

  //setting the target for each wheel using base speed and calculated target speed
  float targetLeft = baseLeft + targetSpeed;
  float targetRight = baseRight - targetSpeed;

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

  //updating the previous counts
  prevLeft = countsLeft;
  prevRight = countsRight;
  interrupts();


  //calculating the error for each wheel
  int16_t errorLeft = targetLeft - speedLeft;
  int16_t errorRight = targetRight - speedRight;

  //calculating the error sum
  sumLeft += errorLeft;
  sumRight += errorRight;

  //using PID algorithm to calcuate effort for each motors using Kp as 10 and Ki as 0.2
  float effortLeft = 10 * errorLeft + 0.2 * sumLeft;
  float effortRight = 10 * errorRight + 0.2 * sumRight;

  motors.setSpeeds(effortLeft, effortRight); // setting motor speeds

}

/*
   Function detectLine() checks if the robot detects the white line using line sensors
*/
bool Robot::detectLine() {
  // Reading the line sensors.
  lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
  if (state == ROBOT_WALL_FOLLOW) { //to be executed only if in ROBOT_WALL_FOLLOW state
    if (lineSensorValues[0] < 150 || lineSensorValues[1] < 150 || lineSensorValues[2] < 150) { //checks if either of the sensors read low values
      lineDetected = true;
    }
    else lineDetected = false;
    return lineDetected;
  }
}

/*
   Function turnTillLine() turns the robot clockwise until it detects the line again
*/
void Robot::turnTillLine() {
  motors.setSpeeds(150, -150);
  if (state == ROBOT_LINE_FOLLOW) {
    if (lineSensorValues[1] < 150) { //checks if line sensor 1 detechs the white line
      state = ROBOT_LINE_FOLLOW; //update the state
      lineNotDetected = false;
      lineEnded = false;
    }
    else lineNotDetected = true;
  }
}

/*
   Function detectIR() checks if the robot detects an ir signal from the remote
*/
bool Robot::detectIR() {
  if (state == ROBOT_LINE_FOLLOW) { //to be executed only if in ROBOT_LINE_FOLLOW state

    decoder.service(); // used to detect the ir signal

    ledRed(decoder.criticalTime()); //turns the red led on if ir signal is received

    if (decoder.criticalTime())
    {
      irDetected = true;
    }
    else
    {

      if (decoder.getAndResetMessageFlag()) // checks if the decoder receives a new message
      {
        lastMessageTimeMs = millis();
        irDetected = true;
      }

      if (decoder.getAndResetRepeatFlag()) //checks if the decoder receives a repeated signal, caused by holding down the remote button
      {
        lastMessageTimeMs = millis();
        irDetected = true;
      }
    }
  }
  return irDetected;
}

/*
   Function rampAngle() detects the change in angle (in Y-direction)
*/
void Robot::rampAngle() {
  if (state == ROBOT_RAMP) { //to be executed only when in ROBOT_RAMP state
    if (filter.calcAngleY(observedAngle, est, gyroBias)) { //call to calcAngleY function in SensorFusion2 class
      if (est > 0.25) { //checks if the estimated angle is over the threshold
        onRamp = true;
        //change the base spped of the wheels
        baseLeft = 25;
        baseRight = 25;
      }
      if ((est < 0.0) && (onRamp == true)) {//checks if the estimated angle is below the threshold
        rampComplete = true;
      }
    }
  }
}

/*
   Function finishTurn() uses gyroscope to check the change in angle (in Z-direction)
*/
void Robot::finishTurn() {
  if (state == ROBOT_360_TURN) {
    motors.setSpeeds(185, -185);
    if (filter.calcAngleZ(observedAngle, est, gyroBias)) {
      if (est < 0) {
        turnedHalfway = true;
      }
      else if (turnedHalfway == true && est > 0.4) {
        state = ROBOT_IDLE;
      }
    }
  }
}

/*
   Function HandleButtonPress() handles calls to different sets of intructions to be performed when button is pressed
*/
void Robot::HandleButtonPress() {
  if (state == ROBOT_IDLE) { //to be excuted only when the robot is in IDLE state
    timer.Start(1000); //wait for 1 second
    state = ROBOT_WAITING; //update the state
  }
}

/*
   Function HandleTimerExpired() handles calls to different sets of intructions to be performed when the timer is expired
*/
void Robot::HandleTimerExpired () {
  if (state == ROBOT_WAITING) { //to be executed only if the robot is in WAITING state
    state = ROBOT_WALL_FOLLOW; //update the state
    timer.Cancel(); //cancel the timer
  }
  else if (state == ROBOT_WALL_FOLLOW) { //to be executed only if in ROBOT_WALL_FOLLOW state
    timer.Cancel(); //cancel the timer
    state = ROBOT_LINE_FOLLOW; //update the state
  }
  else if (state == ROBOT_LINE_FOLLOW) { //to be executed only if in LINE_FOLLOW state
    timer.Cancel(); //cancel the timer
  }
  else if (state == ROBOT_RAMP) { //to be executed only if in TOBOT_RAMP state
    timer.Cancel(); //update the state
    turned = true;
    baseLeft = 25; //setting the left base speed
    baseRight = 28; //setting the right base speed
    targetSpeed = 0; //set the target speed to 0
    prevLeft = countsLeft; //updating the left encoder counts
    prevRight = countsRight; //updating the right encoder counts

  }
  else if (state == ROBOT_360_TURN) { //to be executed only if in ROBOT_360_TURN state
    timer.Cancel(); //cancel the timer
    readyToTurn = true;
  }
}

/*
   Function HandleLineDetected() handles calls to different sets of intructions to be performed when white line is detected
*/
void Robot::HandleLineDetected() {
  if (state == ROBOT_WALL_FOLLOW) { //to be executed only if in WALL_FOLLOW state
    timer.Start(300); //start the timer
    motors.setSpeeds(130, 130); //setting the motor speed to pass the line
  }
}

/*
   Function HandleIrDetected() handles calls to different sets of intructions to be performed when IR signal is detected
*/
void Robot::HandleIrDetected() {
  if (state == ROBOT_LINE_FOLLOW) { //to be executed only if in LINE_FOLLOW state
    timer.Start(800); //start the timer
    motors.setSpeeds(135, -135); //setting the motor speed to turn the robot
    state = ROBOT_RAMP; //updating the state
  }
}

/*
   Function executeStateMachine() is the main function which executes the state machine and determines how the robot will finally work
*/
void Robot::executeStateMachine() {

  // if statements to execute different conditions of state machine:
  if (buttonC.CheckButtonPress()) HandleButtonPress();
  if (timer.CheckExpired()) HandleTimerExpired();
  if (detectLine()) HandleLineDetected();
  if (detectIR()) HandleIrDetected();

  // actual state machine:
  switch (state) {
    case ROBOT_IDLE:
      motors.setSpeeds(0, 0); //stop the motors
      break;
    case ROBOT_WAITING:
      motors.setSpeeds(0, 0); //stop the motors
      break;
    case ROBOT_WALL_FOLLOW:
      if (readyToPID) { //execute PID
        distancePID();
        motorPID();
      }
      break;
    case ROBOT_LINE_FOLLOW:
      if (lineNotDetected) { //check if line is detected
        turnTillLine();
      }
      else if (readyToPID) { //execute PID
        linePID();
        motorPID();
        detectIR();
        lineFinished();
      }
      else if (lineEnded) { //check if line is ended
        state = ROBOT_IDLE; //sets the state to IDLE
      }
      break;
    case ROBOT_RAMP:
      if (turned) { //check if the robot has turned
        if (rampComplete) { //check if the ramp is finished
          timer.Start(2000); //start the timer
          motors.setSpeeds(100, 100); //set the motor speed
          state = ROBOT_360_TURN; //update the state
        }
        else if (readyToPID) { //execute PID
          motorPID();
          rampAngle();
        }
      }
      break;
    case ROBOT_360_TURN:
      if (readyToTurn) { //check if the robot is ready to make 360 turn
        finishTurn();
      }
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
