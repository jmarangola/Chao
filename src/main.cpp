#include <Arduino.h>
#include <MPU6050.h>
#include <cmath>

MPU6050 IMU1;


void setup(){
  Serial.begin(115200);
  Wire.begin(21,22,400000);
  IMU1.initialize();
  IMU1.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
}

int16_t x, y, z;
void loop(){
  IMU1.getRotation(&x, &y, &z);
  Serial.println(y);
}