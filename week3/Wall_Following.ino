// Including Libraries :-

#include <EventTimer.h> // timer class library
#include <button.h> // button class library
#include <Zumo32U4Motors.h> // zumo motors library 
#include <Zumo32U4Encoders.h> // zumo encoder libary
#include <params.h> // PID library
#include <serial_comm.h> // PID library
#include <Wire.h> // Zumo library
#include <Zumo32U4.h> // Zumo library
//====================================================

// defining variables:-

volatile uint16_t pulseStart = 0;
volatile uint16_t pulseEnd = 0;
float targetDistance = 35.0;
float actualDistance = 0;
float targetSpeed = 0;
float baseLeft = 20.0;
float baseRight = 20.0;


volatile int16_t countsLeft = 0; // keeps track of left encoder counts
volatile int16_t countsRight = 0; // keeps track of right encode counts

//this may be any appropriate pin, connect the pin to Trig on the sensor
const uint8_t trigPin = 14;

//for scheduling pings
uint32_t lastPing = 0;
uint32_t PING_INTERVAL = 250; //ms

volatile uint8_t readyToPID = 0;   //a flag that is set when the PID timer overflows


#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];
bool useEmitters = true;
bool lineDetected = false;
//=====================================================

// Creating Objects:-

Button buttonC (17); //button C is pin 17 on the Zumo
EventTimer timer;   //assumes you named your class EventTimer
//use the Pololu libraries for motors
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
//=========================================================

//define the states for the echo capture
enum PULSE_STATE {PLS_IDLE, PLS_WAITING_LOW, PLS_WAITING_HIGH, PLS_CAPTURED};

//and initialize to IDLE
volatile PULSE_STATE pulseState = PLS_IDLE;
//=======================================================================

//define the states for robot
enum ROBOT_STATES {ROBOT_IDLE, ROBOT_WAITING, ROBOT_WALL_FOLLOW, ROBOT_SPINNING};
ROBOT_STATES state;
//==========================================================================

/*
   Commands the MaxBotix to take a reading
*/
void CommandPing(int trigPin) {

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

void distancePID() {
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

    //EDIT THIS LINE: convert pulseLengthTimerCounts, which is in timer counts, to time, in us
    //You'll need the clock frequency and the pre-scaler to convert timer counts to time

    uint32_t pulseLengthUS = pulseLengthTimerCounts * 4; //pulse length in us


    //EDIT THIS LINE AFTER YOU CALIBRATE THE SENSOR
    actualDistance = (pulseLengthUS - 84.519) / 57.252; //distance in cm

    //                Serial.print(millis());
    //                Serial.print('\t');
    //                Serial.print(pulseLengthTimerCounts);
    //                Serial.print('\t');
    //                Serial.print(pulseLengthUS);
    //                Serial.print('\t');
    //                Serial.print(actualDistance);
    //                Serial.print('\n');
  }

  //clear the timer flag
  //  readyToPID = 0;

  noInterrupts();
  float distanceError = actualDistance - targetDistance;
  static float prevDistError = 0;

  static float distanceSum = 0;
  distanceSum += distanceError;

  static float distanceDiff = 0;
  distanceDiff = distanceError - prevDistError;
  //constrain(distanceDiff,1,2);

  targetSpeed =  1 * distanceError - 0.1 * distanceDiff;

  prevDistError = distanceError;
  interrupts();

  //  Serial.println(actualDistance);
  //  Serial.print('\t');
  //  Serial.print(distanceDiff);
  //  Serial.print('\t');
  //  Serial.print(distanceError);
  //  Serial.print('\t');
  //  Serial.print(targetSpeed);
  //  Serial.print('\n');

}

void motorPID() {

  //clear the timer flag
  readyToPID = 0;

  float targetLeft = baseLeft - targetSpeed;
  float targetRight = baseRight + targetSpeed;

  //for tracking previous counts
  static int16_t prevLeft = 0;
  static int16_t prevRight = 0;

  //error sum
  static int16_t sumLeft = 0;
  static int16_t sumRight = 0;

  //  // error diff
  //  static int16_t diffLeft = 0;
  //  static int16_t diffRight = 0;
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
  //  Serial.print(actualDistance);
  //  Serial.print('\t');
  //  Serial.print(targetLeft);
  //  Serial.print('\t');
  //  Serial.print(targetRight);
  //  Serial.print('\n');
  //  Serial.print(effortLeft);
  //  Serial.print('\t');
  //  Serial.print(effortRight);
  //  Serial.print('\n');
  //  Serial.print(millis());

  //        you'll want to add more serial printout here for testing

  if (CheckSerialInput()) {
    ParseSerialInput();
  }


  /* for reading in gain settings
     CheckSerialInput() returns true when it gets a complete string, which is
     denoted by a newline character ('\n'). Be sure to set your Serial Monitor to
     append a newline
  */



}

bool detectLine() {
  // Read the line sensors.
  lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
  if (lineSensorValues[0] < 200 || lineSensorValues[2] < 200 || lineSensorValues[4] < 200) {
    lineDetected = true;
  }
  else lineDetected = false;
  return lineDetected;
}

void HandleButtonPress() {
  if (state == ROBOT_IDLE) {
    Serial.println("In HandleButtonPress()");
    timer.Start(1000);
    Serial.println("timer started");
    state = ROBOT_WAITING;
    Serial.println("state changed");
  }
}

void HandleTimerExpired() {
  Serial.println ("In HandleTimerExpired() ");
  if (state == ROBOT_WAITING) {
    Serial.println("Executed when state in ROBOT_WAITING");
    state = ROBOT_WALL_FOLLOW;
    Serial.println("State changed");
    timer.Cancel();
    Serial.println("Timer cancelled");
  }
  else if (state == ROBOT_SPINNING) {
    state = ROBOT_IDLE;
    Serial.println("Executed when state in ROBOT_SPINNING");
    timer.Cancel();
    Serial.println("Timer cancelled");
    motors.setSpeeds(0, 0);
    Serial.print ("Sets speed back to 0");
  }
}

void HandleDetectLine() {
  if (state == ROBOT_WALL_FOLLOW) {
    state = ROBOT_SPINNING;
    timer.Start(2000);
  }
}


void setup() {
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
  state = ROBOT_IDLE;
  Serial.println("Initialized state as IDLE");

}

void loop() {
  if (buttonC.CheckButtonPress()) HandleButtonPress();
  if (timer.CheckExpired()) HandleTimerExpired();
  if (detectLine()) HandleDetectLine();
  if (timer.CheckExpired()) HandleTimerExpired();


  // Execute State Machine:
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
    case ROBOT_SPINNING:
      motors.setSpeeds(150, -150);
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
