/**
 * Author: John Marangola - marangol@bc.edu
 * 
 **/ 


#include <Arduino.h>
#include <MPU6050.h>
#include <cmath>
#include <PS4Controller.h>
#include <PIDController.h>


#define GYROSCOPE_S 65.6
#define dT_MICROSECONDS 5000
#define dT dT_MICROSECONDS/1000000.0
#define STABLE_STATE_DELTA_THETA 0.05
#define EMA_ALPHA 0.82

#define STEP_L 19
#define DIR_L 33

#define MICROSTEP_RES 1
#define DELTA_T 10000

#define TURN_SENSITIVITY 1.0

// Speed control
#define ALARM_MIN_DELTA 22000
#define ALARM_MAX_DELTA 35000
#define SPEED_DIV 44000.00

// PID constants:
#define kp_a 1.0
#define kd_a 1.0
#define ki_a 1.0

// IMU related:
MPU6050 IMU1;
float gyroscopeOffsets[3], thetaOld;
float joyInput[4] = {0, 0, 0, 0};
long currentTime, lastTime;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// PID Objects
//PIDController angle(kp_a, ki_a, kd_a, (int16_t)(-1*MAXIMUM_SPEED_T), (int16_t) MAXIMUM_SPEED_T, DELTA_T);

// Hardware clock timers
hw_timer_t *leftMotorTimer = NULL;
//hw_timer_t *rightMotorTimer = NULL;
//volatile int thesholdPulses = 4000; // threshold steps around
//volatile int maxpulses = 4000*8;
//volatile int ceiling = thesholdPulses;
volatile int8_t state = 0;
//volatile int steps = 0;
//volatile int timeCount = 16000;
// volatile int timeReducton = 600;


/*const int rampRate = 80;
volatile double rEndV, rV=0;
volatile long rSteps = 0;
volatile int rState = 0;
volatile int rDirection = 1; 
volatile int rLastDirection = rDirection;
volatile bool inTransition = 0;
int rAbsoluteTime = 0.0;

int dS = 400;

void IRAM_ATTR rampLeftStepper() {
  portENTER_CRITICAL_ISR(&timerMux);
  Serial.println(rSteps);
  if (rDirection != rLastDirection) {
    digitalWrite(19, HIGH);
    rSteps = dS;
    inTransition = true;
    rLastDirection = rDirection;
  }
  else if (rSteps == dS) {
    Serial.println("equal");
    if (inTransition) { // decelerating 
      if (rV <= 0) { // finished decelerating
        inTransition = false;
        rV = 0.0;
      }
      else { // Still decelerating
        rAbsoluteTime = floor(50000.0/rV) + rampRate;
        timerAlarmWrite(leftMotorTimer, rAbsoluteTime, true);
        rV = (50000.0/rAbsoluteTime);
      }
    }
    else { // If the direction of the stepper has not been altered
      if (rV < rEndV) { // Current frequency is less than that of the desired setpoint, must ramp up a level
        rAbsoluteTime = floor(50000.0/rV) - rampRate;
        timerAlarmWrite(leftMotorTimer, rAbsoluteTime, true);
        rV = (50000.0/rAbsoluteTime);
      }
      else if (rV > rEndV) { // Current frequency is greater than desired setpoint --> ramp down
        rAbsoluteTime = floor(50000.0/rV) + rampRate;
        timerAlarmWrite(leftMotorTimer, rAbsoluteTime, true);
        rV = (50000.0/rAbsoluteTime);
      }
    }
    rSteps = 0;
  } // Pulse stepper here
  Serial.println("pulsing");
  if (!rState) {
    GPIO.out_w1ts = 1<< 19;
    rState = 1;
  }
  else {
    rState = 0;
    GPIO.out_w1tc = 1<< 19;
  }
  rSteps++;
  portEXIT_CRITICAL_ISR(&timerMux);
}*/

volatile int lastDirection;
volatile int direction = 1;
volatile int previousVel;
volatile long t_ = 12000;
volatile int ticks=0;
volatile int intervalSetpoint = 5000;
volatile int disabled = 0;


void IRAM_ATTR leftTimerFunc() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (!state) {
    GPIO.out_w1ts = 1<< 19;
    state = 1;
  }
  else {
    state = 0;
    GPIO.out_w1tc = 1<< 19;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

double getTimerValue(double velocity) {
  int timerValue = (int) (SPEED_DIV/velocity);
  if (timerValue < ALARM_MIN_DELTA) 
    return ALARM_MIN_DELTA;
  else if (timerValue > ALARM_MAX_DELTA)
    return ALARM_MAX_DELTA;
  return timerValue;
}

/**
 * Update function mutates timer parameters in loop()
 * Velocity is a values between (0, 2)
 **/ 
void updateLeft(double velocity) {
  direction = (velocity > 0) ? 1 : -1;
  velocity = abs(velocity);
  if (lastDirection != direction) {
    digitalWrite(33, direction == 0 ? 0 : 1);
    lastDirection = direction;
  }
  if (previousVel != velocity) {
    if (disabled) {
      disabled = 0; timerAlarmEnable(leftMotorTimer);
    }
    else if (abs(velocity) > 0) 
        timerAlarmWrite(leftMotorTimer, getTimerValue(abs(velocity)), true);
    else if (velocity == 0) 
      disabled = 1; timerAlarmDisable(leftMotorTimer);   
    previousVel = velocity;    
  }
}

void setup() {

  //PS4.begin("01:01:01:01:01:01");
  Serial.begin(115200);
  Serial.println("Ready.");
  pinMode(DIR_L, OUTPUT);
  Wire.begin(21,22,400000);
  IMU1.initialize();
  IMU1.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

  // Setup hardware timers:
  leftMotorTimer = timerBegin(0, 2, true);
  //timerAttachInterrupt(leftMotorTimer, &leftTimerFunc, true);
  timerAttachInterrupt(leftMotorTimer, &leftTimerFunc, true);
  timerAlarmWrite(leftMotorTimer, 22000, true);
  timerAlarmEnable(leftMotorTimer);
  pinMode(19, OUTPUT);
  pinMode(33, OUTPUT);
  digitalWrite(33, HIGH);
  lastTime = millis();
 
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
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

// HPF Coef:
#define c 0.98

double complimentaryAngleFilter(float *accelerometer, float *pitch, float *roll) {
  //
  // angle = c*(angle + gyroYrate*dT) + (1-c) * accXAngle;

}

void getAxisInput(int x0Dz, int y0Dz, int x1Dz, int y1Dz, float axisInput[]){
  double xRaw0, yRaw0, xRaw1, yRaw1;
  if (PS4.isConnected()) {
        if (PS4.event.analog_move.stick.lx) {
            xRaw0 = (double) PS4.data.analog.stick.lx;
            axisInput[0] = (abs(xRaw0) > x0Dz) ? xRaw0 : 0.0;
            //Serial.println(axisInput[0]);
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
 * Tank implementation, modifie integers corresonding to wheel speeds for each wheel
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
  left = (joyInput[1] * maxSpeed) + (TURN_SENSITIVITY * (-1.00/2.00) * joyInput[2]); 
  right = (joyInput[1] * maxSpeed) + (TURN_SENSITIVITY * (1.00/2.00) * joyInput[2]);
}

float accAng = 0.0, thetaAngle = 0.0, thetOld = 0.0;
float speed = 2.0;
float angleOutput = 0.0;

const long DT_MS = 10; // f=100 hz cycle
int time_ = 15000;


void loop(){

}