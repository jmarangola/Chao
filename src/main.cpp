#include <Arduino.h>
#include <MPU6050.h>
#include <cmath>
#include <FastAccelStepper.h>

#define GYROSCOPE_S 65.6
#define dT_MICROSECONDS 5000
#define dT dT_MICROSECONDS/1000000.0
#define STABLE_STATE_DELTA_THETA 0.05
#define EMA_ALPHA 0.80

#define STEP_LEFT_P 19
#define DIR_LEFT__P 2
#define STEP_RIGHT_P 0
#define DIR RIGHT_P 18

// IMU datum
MPU6050 IMU1;
float gyroscopeOffsets[3], thetaOld;

// L/R Nema 17 Motors
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperLeft = NULL;
FastAccelStepper *stepperRight = NULL;

void initMotors(){
  engine.init();
  // Initialize FastAccelStepper instances here:
  stepperLeft = engine.stepperConnectToPin(STEP_LEFT_P);
  //stepperRight = engine.stepperConnectToPin(STEP_RIGHT_P);
  stepperLeft->setAutoEnable(true);
  //stepperRight->setAutoEnable(true);
}

void setup(){
  Serial.begin(115200);
  Wire.begin(21,22,400000);
  IMU1.initialize();
  IMU1.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  initMotors();
  stepperLeft->setSpeed(400);  // the parameter is us/step !!!
  stepperLeft->setAcceleration(50000);
  stepperLeft->rampState();
  stepperLeft->move(10000);

}


//float dT = 0.005;
float deltaGyroAngle, accAngle;
float gXSum = 0.0;
int16_t gyrX, gyrY, gyrZ, aX, aY, aZ, oX, oY, oZ;

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
  
  stepperLeft->setCurrentPosition(0);
  stepperLeft->move(100);

}