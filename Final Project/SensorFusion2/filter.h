#include "Arduino.h"
#include <stdint.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <math.h>

#ifndef __FILTER_H
#define __FILTER_H

class ComplementaryFilter {

  private:
    LSM303 compass;
    L3G gyro;
    float estimatedAngle; 
    float accOffset; // offset for accelerometer
    float gyroBias = 0; // initializing the gyroBias to 0
    float lastReading = 0; // initializing the previous readings to 0
    float gamma; // gyroscope readings
    float loopRate = 0.01; // to calculate deltaT
    float angleSum; // to calculate the average
    float lastBias; // to update the gyroBias
    float eps = 0.01; // value of "epsilon" 
    float k = 0.5; // value of "kappa"
    float fusedAngle; // to store the fused estimate
    bool angleFlag = false; // setting a flag to check calcAngle()
    bool average = true;

    public:
    ComplementaryFilter(LSM303 compass,L3G gyro,float estimatedAngle, float accOffset, float gyroBias);
    void Init(void);
    bool calcAngleY(float& observedAngle, float& est, float& gyroBias);
    bool calcAngleZ(float& observedAngle, float& est, float& gyroBias);
};

#endif
