#include <Arduino.h>
#include <stdint.h>
#include <button.h>// button class library
#include <EventTimer.h>// timer class library
#include <Zumo32U4Motors.h> // zumo motors library 
#include <Zumo32U4Encoders.h> // zumo encoder libary

#include <Wire.h> // Zumo wire library
#include <Zumo32U4.h> // Zumo robot library

#ifndef __ROBOT_H
#define __ROBOT_H
#define NUM_SENSORS 5

class Robot {

  public:
    //private:
    // defining robot states:
    enum ROBOT_STATES {ROBOT_IDLE, ROBOT_WAITING, ROBOT_WALL_FOLLOW,ROBOT_TURN_90, ROBOT_LINE_FOLLOW, ROBOT_RAMP, ROBOT_360_TURN};
    ROBOT_STATES state = ROBOT_IDLE;// initialized to IDLE

    // defining variables:-
   

    //Creating objects:

    EventTimer timer;


    // public:
    Robot();
    void Init();
    void CommandPing(int trigPin);
    void distancePID();
    void motorPID();
    bool detectLine();
    bool detectIR();
    bool detectHorizon();
    void turnTillLine();
    //void HandleTurn();
    void HandleButtonPress();
    void HandleTimerExpired();
    void HandleLineDetected();
    void HandleIrDetected();
    void HandleHorizonDetected();
    void executeStateMachine();
};
#endif
