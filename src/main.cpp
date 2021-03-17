#include "Wire.h" // For I2C communication
#include <Arduino.h>
#include <cmath>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
double old_value, new_value = 0;
double filtered = 0.0;

int16_t temperature; 
char tmp_str[7]; 

char* convert_int16_to_str(int16_t i) { // Converts int16 to string of uniform length
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

double get_raw_IMU_X(){
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false); 
    Wire.requestFrom(MPU_ADDR, 7*2, true); 
    gyro_x = Wire.read()<<8 | Wire.read();
    new_value = double(gyro_x);
    filtered = new_value * 0.01 + old_value * 0.99;
    //Serial.print(" | gZ = "); Serial.print(filtered);
    old_value = new_value;
    return filtered;
}

// Returns the average of an n-sample of imu x-axis data
double n_sample_IMU_X(int n){ 
    double sampleSumX = 0.0;
    for (int i = 0; i < n; i++){
        sampleSumX += get_raw_IMU_X();
    }
    return (sampleSumX / n);
}

//onst double PI = 2.0*acos(0.0);
const double RAD_TO_D = 180.0/PI;
const int minVal = -16000, maxVal = 16000;

void getIMUData(float *accelDat, float *gyroDat) {
    float accelerationV[2], gyroV[2];
    int16_t accRawX, accRawY, accRawZ, gyrRawX, gyrRawY, gyrRawZ;
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true);
    accRawX = Wire.read() << 8 | Wire.read(); 
    accRawY = Wire.read() << 8 | Wire.read();
    accRawZ = Wire.read() << 8 | Wire.read(); 
    Wire.beginTransmission(0x68);
    Wire.write(0x43); 
    Wire.endTransmission(false);

    Wire.requestFrom(0x68,4,true);
    gyrRawX = Wire.read() << 8|Wire.read(); 
    gyrRawY = Wire.read() << 8|Wire.read(); 
    int xAng = map(accRawX,minVal,maxVal,-90,90);
    int yAng = map(accRawY,minVal,maxVal,-90,90);
    int zAng = map(accRawZ,minVal,maxVal,-90,90);
 
    gyroV[0] = RAD_TO_D * (atan2(-yAng, -zAng)+PI);
    Serial.println(RAD_TO_D * (atan2(-xAng, -zAng)+PI));
    gyroV[2] = RAD_TO_D * (atan2(-yAng, -xAng)+PI);
    gyroDat = gyroV;
    accelDat = accelerationV;
}

double GYROSCOPE_SENSITIVITY = 65.536;
double ACCELEROMETER_SENSITIVITY = 8192.0;
double dt = 0.01;
//double M_PI = 2*acos(0.0);
/*
void ComplementaryFilter(float accData[3], float gyrData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc;               
    
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
 
    // Compensate for drift with accelerometer data if !garbage data
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
	    // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
 
	    // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }
} */

void setup() {
  Serial.begin(115200);
  Serial.println("here");
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begin transmission to the I2C slave (GY-521)
  Wire.write(0x6B);
  Wire.write(0); //  Wake up the MPU-6050
  Wire.endTransmission(true);  
}

float gDat[2], aDat[2]; 
void loop() {
  Serial.println(n_sample_IMU_X(20));
  delay(1);
}