#include "filter.h"

LSM303 compass;
L3G gyro;
float estimatedAngle;
float accOffset;
float gyroBias;
float observedAngle;
float est;
ComplementaryFilter filter (compass, gyro, estimatedAngle, accOffset, gyroBias);



char report[120];

void setup()
{
  Serial.begin(115200);
  filter.Init();
}

void loop()
{
//  if (filter.calcAngleY(observedAngle,est,gyroBias)){
//    Serial.print(observedAngle);
//    Serial.print('\t');
//    Serial.print(est);
//    Serial.print('\t');
//    Serial.print(gyroBias);
//    Serial.print('\n');

//  }

  if (filter.calcAngleZ(observedAngle,est,gyroBias)){
    Serial.print(observedAngle);
    Serial.print('\t');
    Serial.print(est);
    Serial.print('\t');
    Serial.print(gyroBias);
    Serial.print('\n');
  }

}
