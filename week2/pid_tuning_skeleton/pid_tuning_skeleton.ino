#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4ButtonA buttonA;
Zumo32U4ButtonC buttonC;

//const uint8_t buttonPin = 14; //button A on the Romi
//
//Button buttonA(buttonPin);
/*
 * Code for using TCC4 for precision PID timing.
 * You'll need to set TOP to set the interval
 * 
 * This code adds the ability to tune the gains and change the targets
 */

#include <Zumo32U4Motors.h>
#include <Zumo32U4Encoders.h>

#include "params.h"
#include "serial_comm.h"

volatile uint8_t readyToPID = 0;   //a flag that is set when the PID timer overflows

Zumo32U4Motors motors;

Zumo32U4Encoders encoders;
volatile int16_t countsLeft = 0;
volatile int16_t countsRight = 0;

void setup()
{
  Serial.begin(115200);
  while(!Serial) {}  //IF YOU DON'T COMMENT THIS OUT, YOU MUST OPEN THE SERIAL MONITOR TO START
  Serial.println("Hi.");

  noInterrupts(); //disable interupts while we mess with the Timer4 registers
  
  //sets up timer 4
  TCCR4A = 0x00; //disable some functionality -- no need to worry about this
  TCCR4B = 0x0C; //sets the prescaler -- look in the handout for values
  TCCR4C = 0x04; //toggles pin 6 at one-half the timer frequency
  TCCR4D = 0x00; //normal mode

  OCR4C = 124;   //TOP goes in OCR4C
  TIMSK4 = 0x04; //enable overflow interrupt
  
  interrupts(); //re-enable interrupts

//  buttonA.Init();

   //pinMode(6, OUTPUT); //COMMENT THIS OUT TO SHUT UP THE PIEZO!!!
}

void loop() 
{    

  if (buttonC.isPressed()) {
    //targetLeft = 50 * 1;
    //targetRight = 50* 1;
  }
//  else if (buttonC.isPressed()) {
   // targetLeft = 25;
   // targetRight = 25;
//  }
  
  if(readyToPID) //timer flag set
  {
    //clear the timer flag
    readyToPID = 0;

    //for tracking previous counts
    static int16_t prevLeft = 0;
    static int16_t prevRight = 0;

    //error sum
    static int16_t sumLeft = 0;
    static int16_t sumRight = 0;

    /*
     * Do PID stuffs here. Note that we turn off interupts while we read countsLeft/Right
     * so that it won't get accidentally updated (in the ISR) while we're reading it.
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

    //you'll want to add more serial printout here for testing
    Serial.print(targetLeft);
    Serial.print('\t');
    Serial.print(speedLeft);
    Serial.print('\t');
    Serial.print(effortLeft);
    Serial.print('\n');

//    Serial.print(targetRight);
//    Serial.print('\t');
//    Serial.print(speedRight);
//    Serial.print('\t');
//    Serial.print(effortRight);
//    Serial.print('\n');
    
    //Serial.print(countsLeft);


    //Serial.print('\n');
  }

  /* for reading in gain settings
   * CheckSerialInput() returns true when it gets a complete string, which is
   * denoted by a newline character ('\n'). Be sure to set your Serial Monitor to 
   * append a newline
   */
  if(CheckSerialInput()) {ParseSerialInput();}
}

/*
 * ISR for timing. Basically, raise a flag on overflow. Timer4 is set up to run with a pre-scaler 
 * of 1024 and TOP is set to 249. Clock is 16 MHz, so interval is dT = (1024 * 250) / 16 MHz = 16 ms.
 */
ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();

  readyToPID = 1;
}
