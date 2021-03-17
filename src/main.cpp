#include <Arduino.h>
#include <MPU6050.h>
#include <cmath>

#define GYROSCOPE_S 65.6
#define dT_MICROSECONDS 5000
#define dT dT_MICROSECONDS/1000000.0

MPU6050 IMU1;


void setup(){
  Serial.begin(115200);
  Wire.begin(21,22,400000);
  IMU1.initialize();
  IMU1.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
}




//float dT = 0.005;
float deltaGyroAngle, accAngle;
float gXSum = 0.0;
int16_t gyrX, gyrY, gyrZ, aX, aY, aZ, oX, oY, oZ;

void loop(){
  for (int i = 0; i <= 100; i++){
    IMU1.getRotation(&oX, &oY, &oZ);
    gXSum += oX;
    delay(5);
  }
  gXSum /= 100;
  IMU1.getMotion6(&gyrX, &gyrY, &gyrZ, &aX, &aY, &aZ);
  accAngle = atan2f((float) aY, (float) aZ) * 180.0/(2.0*acos(0.0)) - 2;
  deltaGyroAngle = ((float)((gyrX - gXSum)) / GYROSCOPE_S) * dT;
  Serial.println(deltaGyroAngle);
}