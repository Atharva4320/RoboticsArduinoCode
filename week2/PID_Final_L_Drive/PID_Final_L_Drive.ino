// including PID libraries
#include <params.h>
#include <serial_comm.h>
//================================

#include <button.h>//include your button class from last week
#include <EventTimer.h> //include your shiny, new event timer class
#include "Segment.h" // include Segment library

#include <Zumo32U4Motors.h>
#include <Zumo32U4Encoders.h>


volatile uint8_t readyToPID = 0;   //a flag that is set when the PID timer overflows

volatile int16_t countsLeft = 0; // keeps track of left encoder counts
volatile int16_t countsRight = 0; // keeps track of right encode counts
volatile int16_t bufferCount = 0; // variable to track total counts after each segment

volatile int16_t differenceL;
volatile int16_t differenceR;

int16_t countPerCM = 48.97075172;

volatile int16_t totalCounts = 0;

volatile int16_t distance;
int Seg = 0;
const int numberOfSegments = 3; //fwd, turn, fwd => so 3 segments for now
Segment segments [numberOfSegments];

// instantialting segments

Segment one = {20, 20, 60};
Segment two = {20, 0, 18};
Segment three = {20, 20, 40};


Button buttonA(14); //button A is pin 14 on the Zumo
EventTimer timer;   //assumes you named your class EventTimer

//use the Pololu libraries for motors and encoders
Zumo32U4Motors motors;
Zumo32U4Encoders encoders; //(we're not acutally using this in this code, but we will soon)

//declare the robot states here

enum ROBOT_STATES {ROBOT_IDLE, ROBOT_DRIVING};
ROBOT_STATES state;

void Drive (int iSeg) {
  Segment currentSegment = segments [iSeg];
  targetLeft = currentSegment.motorL; // left target speed
  targetRight = currentSegment.motorR; // right target speed
  distance = currentSegment.dis; // total distance needed to travel
  totalCounts =  bufferCount + distance * countPerCM; // total number of encoder counts for the given distance

  differenceL = totalCounts - countsLeft;
  differenceR = totalCounts - countsRight;

  state = ROBOT_DRIVING;
}


void noDrive () { // Stops the motors
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
}


//add you handlers here:

//handle the button press -

void handleButtonPress() { // Handles methods executed when the button is pressed
  if (state == ROBOT_IDLE) {
    state = ROBOT_DRIVING;
    Drive(Seg);
  }
}

//handle the segment change -

void handleSegmentChange () {

  if (state == ROBOT_DRIVING) { // Handles methods executed when difference is below 5

    if (Seg >= 3) {
      state = ROBOT_IDLE;
      noDrive ();
    }
    else {
      Seg++;
      Serial.println (Seg);
      bufferCount = totalCounts; // stores the total count
      Drive(Seg);
    }
  }
}

ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();

  readyToPID = 1;
}

void pidLoop() {


  //clear the timer flag
  readyToPID = 0;

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

  int16_t errorLeft = targetLeft - speedLeft;
  sumLeft += errorLeft;

  int16_t errorRight = targetRight - speedRight;
  sumRight += errorRight;

  float effortLeft = Kp * errorLeft + Ki * sumLeft;
  float effortRight = Kp * errorRight + Ki * sumRight;

  motors.setSpeeds(effortLeft, effortRight); //up to you to add the right motor

  //Serial.print(millis());

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

void setup() {

  noInterrupts(); //disable interupts while we mess with the Timer4 registers

  //sets up timer 4
  TCCR4A = 0x00; //disable some functionality -- no need to worry about this
  TCCR4B = 0x0C; //sets the prescaler -- look in the handout for values
  TCCR4C = 0x04; //toggles pin 6 at one-half the timer frequency
  TCCR4D = 0x00; //normal mode

  OCR4C = 124;   //TOP goes in OCR4C
  TIMSK4 = 0x04; //enable overflow interrupt

  interrupts(); //re-enable interrupts
  // populating the segments:-

  segments [0] = one;
  segments [1] = two;
  segments [2] = three;

  Serial.begin(115200);
  Serial.println("Hello.");

  buttonA.Init(); //don't forget to call Init()!
  state = ROBOT_IDLE;

}

void loop()
{

  if (buttonA.CheckButtonPress()) {
    Serial.println("Button Pressed");
         handleButtonPress();
  }

  switch (state) {

    case ROBOT_IDLE :

      noDrive ();
      break;

    case ROBOT_DRIVING :
      if ((differenceL < -5 ) || (differenceR < -5)) {
        Serial.println ("Time to Stop");
        handleSegmentChange ();
        break;
      }
      else Drive (Seg);
      break;

  }

  if (readyToPID) //timer flag set
  {
    pidLoop();
  }
}
