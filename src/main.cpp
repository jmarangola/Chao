#include "Wire.h" // For I2C communication
#include <Arduino.h>

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
    double sample_sum;
    for (int i = 0; i < n; i++)
        sample_sum += get_raw_IMU_X();
    return (sample_sum / n)
    
}

void setup() {
  Serial.begin(9600);
  Serial.println("here");
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begin transmission to the I2C slave (GY-521)
  Wire.write(0x6B);
  Wire.write(0); //  Wake up the MPU-6050
  Wire.endTransmission(true);  
}

void loop() {
  Serial.println(get_raw_IMU_X());
  delay(50);
}