#include "filter.h"


ComplementaryFilter::ComplementaryFilter(LSM303 compass, L3G gyro, float estimatedAngle, float accOffset, float gyroBias) {
  compass = compass;
  gyro = gyro;
  estimatedAngle = estimatedAngle;
  accOffset = accOffset;
  gyroBias = gyroBias;
}

void ComplementaryFilter::Init(void) {

  Wire.begin();

  if (!compass.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while (1)
    {
      Serial.println(F("Failed to detect the compass."));
      delay(100);
    }
  }

  compass.enableDefault();

  if (!gyro.init())
  {
    // Failed to detect the gyro.
    ledRed(1);
    while (1)
    {
      Serial.println(F("Failed to detect gyro."));
      delay(100);
    }
  }

  gyro.enableDefault();

 // while (!Serial) {} //wait for the Serial Monitor
  compass.writeReg(LSM303 :: CTRL1, 0x67); // setting the sampling frequency of the accelerometer to 100 Hz 
  uint8_t ctrl1 = compass.readReg(LSM303::CTRL1);
//  Serial.print("CTRL1 is: ");
//  Serial.println(ctrl1, HEX);
  gyro.writeReg(L3G :: CTRL1, 0xBF); // setting the gyroscope sampling frequency rate to 400 Hz with a 110 Hz cut-off

}

bool ComplementaryFilter::calcAngleY(float& observedAngle, float& est, float& gyroBias) {
  byte statusA = compass.readReg(LSM303::STATUS_A); // storing the STATUS_A register bites of accelerometer
  byte normalValue = B00001000; 
  if (average) { // checks for average
    for (int i = 0; i < 200; i++) { 
      compass.readAcc();
      observedAngle = atan2(compass.a.x,compass.a.z);
      angleSum += observedAngle;
    }
    accOffset = angleSum/200; // taking an average of 200 readins to calculate the accelerometer offset
    average = false;
 }
  if ((statusA & normalValue) == normalValue) { // bit wise & operator to check each byte
    angleFlag = true; // set the boolean flag to true
    compass.readAcc();
    gyro.read();
    gamma = (0.00875 * (M_PI/180)) * gyro.g.y; // calculating the gyroscope reading
    observedAngle = atan2(compass.a.x,compass.a.z); 
    observedAngle -= accOffset; // substracting the accelerometer offset from readings
    gyroBias = lastBias + eps * (est - observedAngle); // calculating the bias for gyroscope
    est = lastReading + (loopRate * gamma); // calculating the estimated angle
    fusedAngle = k * (est) + (1-k)* (observedAngle); // calculating the fused angle using "kappa"
    lastReading = fusedAngle; // updating the lastReading with fusedAngle
    lastBias = gyroBias; // updating the lastBias

  }
  else {
    angleFlag = false;
  }
  return angleFlag;
}

bool ComplementaryFilter::calcAngleZ(float& observedAngle, float& est, float& gyroBias) {
  byte statusA = compass.readReg(LSM303::STATUS_A); // storing the STATUS_A register bites of accelerometer
  byte normalValue = B00001000; 
  if (average) { // checks for average
    uint8_t readingSum = 0;
    for (int i = 0; i < 100; i++) {
      gyro.read();
      uint8_t gyroReading = (0.00875 * (M_PI/180)) * gyro.g.z;
       readingSum += gyroReading;  
    }
    gyroBias = readingSum/100;
    average = false;
 }
  if ((statusA & normalValue) == normalValue) { // bit wise & operator to check each byte
    angleFlag = true; // set the boolean flag to true
    compass.readAcc();
    gyro.read();
    gamma = (0.00875 * (M_PI/180)) * gyro.g.z; // calculating the gyroscope reading
    observedAngle = atan2(-compass.a.y,compass.a.x); 
    observedAngle -= accOffset; // substracting the accelerometer offset from readings
    //gyroBias = lastBias + eps * (est - observedAngle); // calculating the bias for gyroscope
    est = lastReading + (loopRate * gamma); // calculating the estimated angle
    fusedAngle = 0.9 * (est) + (1-0.9)* (observedAngle); // calculating the fused angle using "kappa"
    lastReading = fusedAngle; // updating the lastReading with fusedAngle
    //lastBias = gyroBias; // updating the lastBias

  }
  else {
    angleFlag = false;
  }
  return angleFlag;
}
