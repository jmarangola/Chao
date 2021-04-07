/**
 * Author: John Marangola - marangol@bc.edu
 * 
 **/ 

#include <Arduino.h>
#include <MPU6050.h>
#include <cmath>
#include <PS4Controller.h>
#include <PIDController.h>
#include <chaoStepper.h>

#define GYROSCOPE_S 65.6
#define dT_MICROSECONDS 5000
#define dT dT_MICROSECONDS/1000000.0
#define STABLE_STATE_DELTA_THETA 0.05
#define EMA_ALPHA 0.82

#define STEP_LEFT_P 19
#define DIR_LEFT_P 2
#define STEP_RIGHT_P 0
#define DIR RIGHT_P 18
#define MICROSTEP_RES 8
#define DELTA_T 10000

#define TURN_SENSITIVITY 1.0

// PID constants:
#define kp_a 1.0
#define kd_a 1.0
#define ki_a 1.0

// IMU related:
MPU6050 IMU1;
float gyroscopeOffsets[3], thetaOld;
float joyInput[4] = {0, 0, 0, 0};
long currentTime, lastTime;

// PID Objects
//PIDController angle(kp_a, ki_a, kd_a, (int16_t)(-1*MAXIMUM_SPEED_T), (int16_t) MAXIMUM_SPEED_T, DELTA_T);

// Hardware clock timers
//hw_timer_t *leftMotorTimer = NULL;
//hw_timer_t *rightMotorTimer = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR leftStepperTrigger() {
  portENTER_CRITICAL_ISR(&timerMux);
  // call func left stepper
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR rightStepperTrigger() {
  portENTER_CRITICAL_ISR(&timerMux);
  // call func here
  portENTER_CRITICAL_ISR(&timerMux);
}


void setup() {

  //PS4.begin("01:01:01:01:01:01");
  Serial.begin(115200);
  Serial.println("Ready.");
  pinMode(DIR_LEFT_P, OUTPUT);
  Wire.begin(21,22,400000);
  IMU1.initialize();
  IMU1.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

  // Setup hardware timers:
  //leftMotorTimer = timerBegin(0, 80, true);
  //timerAttachInterrupt(leftMotorTimer, &leftTimerFunc, true);
  //timerAlarmWrite(leftMotorTimer, 490, true);
  //timerAlarmEnable(leftMotorTimer);
  ///pinMode(19, OUTPUT);
  //pinMode(33, OUTPUT);
  //digitalWrite(33, HIGH);

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

/**
 * EMA Moving Average Low Pass filter implementation, thetaOld is last value, thetaPrimeFiltered is the 
 *  resulting value, dT is the differential time unit
 **/
void emaLowPass(float &accAngle, float &thetaPrimeFiltered, float thetaOld){
  IMU1.getMotion6(&gyrX, &gyrY, &gyrZ, &aX, &aY, &aZ);
  accAngle = atan2f((float) aY, (float) aZ) * 180.0/(2.0*acos(0.0)) - 2;
  thetaPrimeFiltered = ((float)((gyrX - gyroscopeOffsets[0])) / GYROSCOPE_S) * dT;
  thetaPrimeFiltered = ((EMA_ALPHA * thetaOld) +  thetaPrimeFiltered * (1 - EMA_ALPHA));
}

void getAxisInput(int x0Dz, int y0Dz, int x1Dz, int y1Dz, float axisInput[]){
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
        for (int i = 0; i < 4; i++){
            if (axisInput[i] == 0)
                continue;
            axisInput[i] = (128 - abs(axisInput[i]))/128;
            if (axisInput[i] < 0)
              axisInput[i] *= -1;
        }
    }
    else {
      //Serial.println("Joystick not connected.");
    }
}


/**
 * Tank drive implementation, modifie integers corresonding to wheel speeds for each wheel
 * Max speed for 1/2 microstepping resolution
 **/ 
void tankDrive(int &leftSpeed, int &rightSpeed, int maxSpeed) {
  getAxisInput(5, 5, 5, 5, joyInput);
  leftSpeed = joyInput[1] * maxSpeed;
  rightSpeed = joyInput[3] * maxSpeed;
}

/**
 * Simple slide drive implementation
 * Max Speed for 1/2 stepping
 **/ 
void slideDrive(int &left, int &right, int maxSpeed) {
  getAxisInput(5, 5, 5, 5, joyInput);
  maxSpeed *= (2/MICROSTEP_RES);
  // Distribute turning offset:
  left = (joyInput[1] * maxSpeed) + (TURN_SENSITIVITY * (-1/2) * joyInput[2]); 
  right = (joyInput[1] * maxSpeed) + (TURN_SENSITIVITY * (1/2) * joyInput[2]);
}

float accAng = 0.0, thetaAngle = 0.0, thetOld = 0.0;
float speed = 2.0;
float angleOutput = 0.0;

const long DT_MS = 10; // f=100 hz cycle
long tn, tlast = millis();

void loop(){

}