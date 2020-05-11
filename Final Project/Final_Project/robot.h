#include <Arduino.h>
#include <stdint.h>
#include <button.h>// button class library
#include <EventTimer.h>// timer class library
#include <Zumo32U4Motors.h> // zumo motors library 
#include <Zumo32U4Encoders.h> // zumo encoder libary
#include <filter.h>
#include <RemoteDecoder.h>
#include <RemoteConstants.h>

#include <Wire.h> // Zumo wire library
#include <Zumo32U4.h> // Zumo robot library

#ifndef __ROBOT_H
#define __ROBOT_H
#define NUM_SENSORS 3

class Robot {

  private:
    // defining robot states:
    enum ROBOT_STATES {ROBOT_IDLE, ROBOT_WAITING, ROBOT_WALL_FOLLOW, ROBOT_TURN_90, ROBOT_LINE_FOLLOW, ROBOT_RAMP, ROBOT_360_TURN};
    ROBOT_STATES state = ROBOT_IDLE;// initialized to IDLE

    // defining variables:-
    
    //initializing base motor speeds:
    float baseLeft = 10.0;
    float baseRight = 10.0;
    float targetSpeed = 0; //initializing the taget speed at 0
    
    uint16_t lineSensorValues[NUM_SENSORS]; //initializing the number of line sensrs used
    bool useEmitters = true; //  to use the emmiters of the line sensors
    bool lineNotDetected = true; // flag to check if the line has been reached
    bool lineDetected = false; //boolean to check if line is detected
    bool lineEnded = false; //to check if line is ended 
    bool irDetected = false; //to check if IR signal is detected
    bool turned = false; //to check if the robot has turned
    bool onRamp = false; //to check if the robot is on ramp
    bool rampComplete = false; //to check if the robot has completed the ramp
    bool readyToTurn = false; //to check if the robot is ready to turn 
    bool turnedHalfway = false; //check to see if the robot has turned halfway
    // Remote Control variables:
    bool messageActive = false;
    uint16_t lastMessageTimeMs = 0;
    const uint16_t messageTimeoutMs = 115;


  public:
    Robot();
    void Init();
    void CommandPing(int trigPin);
    void distancePID();
    void linePID();
    bool lineFinished();
    void motorPID();
    bool detectLine();
    void turnTillLine();
    bool detectIR();
    void rampAngle();
    void finishTurn();
    void HandleButtonPress();
    void HandleTimerExpired();
    void HandleLineDetected();
    void HandleIrDetected();
    void executeStateMachine();


};
#endif
