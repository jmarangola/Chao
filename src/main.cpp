#include <Arduino.h>
#include <MPU6050.h>
#include <cmath>
#include <PS4Controller.h>
#include <fastStepper.h>

#define GYROSCOPE_S 65.6
#define dT_MICROSECONDS 5000
#define dT dT_MICROSECONDS/1000000.0
#define STABLE_STATE_DELTA_THETA 0.05
#define EMA_ALPHA 0.80

#define STEP_LEFT_P 19
#define DIR_LEFT_P 2
#define STEP_RIGHT_P 0
#define DIR RIGHT_P 18
#define JOY_LOWER -128
#define JOY_UPPER 128

void loop1();
// IMU datum
MPU6050 IMU1;
double axisInput[4] = {0, 0, 0, 0};
float gyroscopeOffsets[3], thetaOld;

void IRAM_ATTR leftTimer();
void IRAM_ATTR motRightTimerFunction();
fastStepper leftStepper(2, 19, 0, leftTimer);
int maxStepSpeed = 1500;


portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR leftTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  leftStepper.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}


void setup(){
  leftStepper.speed = 1000;

  pinMode(STEP_LEFT_P, OUTPUT);

  Serial.begin(115200);

  // Setup joystick
  PS4.begin("01:01:01:01:01:01");
  Serial.println("Ready.");

  Wire.begin(21,22,400000);
  IMU1.initialize();
  IMU1.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

}


//float dT = 0.005;
float deltaGyroAngle, accAngle;
float gXSum = 0.0;
int16_t gyrX, gyrY, gyrZ, aX, aY, aZ, oX, oY, oZ;

void getAxisInput(int x0Dz, int y0Dz, int x1Dz, int y1Dz, double axisInput[]){
  double xRaw0, yRaw0, xRaw1, yRaw1;
  if (PS4.isConnected()) {
        if (PS4.event.analog_move.stick.lx) {
            xRaw0 = (double) PS4.data.analog.stick.lx;
            axisInput[0] = (abs(xRaw0) > x0Dz) ? xRaw0 : 0.0;
            Serial.println(axisInput[0]);
        }
        if (PS4.event.analog_move.stick.ly) {
            yRaw0 = (double) PS4.data.analog.stick.lx;
            axisInput[1] = (abs(yRaw0) > y0Dz) ? yRaw0 : 0.0;
        }
        if (PS4.event.analog_move.stick.rx) {
            xRaw1 = (double) PS4.data.analog.stick.rx;
            axisInput[2] = (abs(xRaw1) > x1Dz) ? xRaw1 : 0.0;
        }
        if (PS4.event.analog_move.stick.ry) {
            yRaw1 = (double) PS4.data.analog.stick.ry; 
            axisInput[3] = (abs(yRaw1) > y1Dz) ? yRaw1 : 0.0;
        }
        // Normalize joystick inputs to percentage of maximum velocity by X: Z -> R [-1, 1]
        /*
        for (int i = 0; i < 4; i++){
            if (axisInput[i] == 0)
                continue;
            axisInput[i] = (axisInput[i] >= 0) ? (double)(JOY_UPPER - axisInput[i])/JOY_UPPER : (double)(JOY_LOWER - axisInput[i])/JOY_LOWER;
        }*/
    }
    else {
      //Serial.println("Joystick not connected.");
    }
}

void computeGyroOffsets(int8_t N = 100) {
  for (int i = 0; i <= 100; i++){
    IMU1.getRotation(&oX, &oY, &oZ);
    gyroscopeOffsets[0] += oX;
    gyroscopeOffsets[1] += oY;
    gyroscopeOffsets[2] += oZ;
    delay(5);
  }
  gyroscopeOffsets[0] /= N;
  gyroscopeOffsets[1] /= N;
  gyroscopeOffsets[2] /= N;
}

void emaLowPass(float &accAngle, float &thetaPrimeFiltered, float thetaOld){
  IMU1.getMotion6(&gyrX, &gyrY, &gyrZ, &aX, &aY, &aZ);
  accAngle = atan2f((float) aY, (float) aZ) * 180.0/(2.0*acos(0.0)) - 2;
  thetaPrimeFiltered = ((float)((gyrX - gyroscopeOffsets[0])) / GYROSCOPE_S) * dT;
  thetaPrimeFiltered = ((EMA_ALPHA * thetaOld) +  thetaPrimeFiltered * (1 - EMA_ALPHA));
}

float accAng = 0.0, thetaAngle = 0.0, thetOld = 0.0;
void loop(){
  /*computeGyroOffsets();
  emaLowPass(accAngle, thetaAngle, thetOld);
  Serial.println(thetaAngle);
  thetOld = thetaAngle;*/
  leftStepper.update();
  


  
  
}