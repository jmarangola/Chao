/**
 * Author: John Marangola - marangol@bc.edu
 * 
 **/ 

#define _USE_MATH_DEFINES

#include <Arduino.h>
#include <MPU6050.h>
#include <cmath>
#include <PS4Controller.h>
#include <PIDController.h>
#include <FastAccelStepper.h>

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *leftStepper = NULL;
FastAccelStepper *rightStepper = NULL;

#define GYROSCOPE_S 65.6
#define DT_MILLIS 20
// 50 hertz
#define dT (DT_MILLIS/1000.0)
#define EMA_ALPHA 0.82

#define STEP_L 19
#define DIR_L 33
#define MICROSTEP_RES 1

#define TURN_SENSITIVITY 1.0

// Speed control
#define ALARM_MIN_DELTA 22000
#define ALARM_MAX_DELTA 35000
#define SPEED_DIV 44000.00

// PID constants:
#define kp_a 0.1
#define kd_a 0.0
#define ki_a 0.0000

// IMU related:
MPU6050 IMU1;
float pitch = 0.0, roll = 0.0, pitchAcc = 0.0, rollAcc = 0.0;
int mag = 0;
float gyroscopeOffsets[3], accelerometerData[3], thetaOld;
float joyInput[4] = {0, 0, 0, 0};
long currentTime, lastTime;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// PID Objects
PIDController anglePID(kp_a, ki_a, kd_a, -2.0, 2.0, (DT_MILLIS/1000.0));

// Hardware clock timers
//hw_timer_t *leftMotorTimer = NULL;
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

double gyroXRate;
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
    GPIO.out_w1ts = 1 << 19;
    GPIO.out_w1ts = 1 << 2;
    state = 1;
  }
  else {
    state = 0;
    GPIO.out_w1tc = 1 << 19;
    GPIO.out_w1tc = 1 << 2;
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
/*
void updateLeft(double velocity) {
  direction = (velocity > 0) ? 1 : -1;
  //Serial.println(direction);
  velocity = abs(velocity);
  if (lastDirection != direction) {
    digitalWrite(33, direction == -1 ? LOW : HIGH);
    digitalWrite(15, direction == -1 ? LOW : HIGH);
    lastDirection = direction;
  }
  if (previousVel != velocity) {
    if (disabled) {
      disabled = 0; 
      timerAlarmEnable(leftMotorTimer); 
    }
    if (abs(velocity) > 0) 
          timerAlarmWrite(leftMotorTimer, getTimerValue(abs(velocity)), true);
    else {
        timerAlarmWrite(leftMotorTimer, 100000, true);
        timerAlarmDisable(leftMotorTimer);
        disabled = 1;

    }
    
  }
  previousVel = velocity;  
}*/



void setup() {

  //PS4.begin("01:01:01:01:01:01");
  Serial.begin(115200);
  Serial.println("Ready.");
  pinMode(DIR_L, OUTPUT);
  Wire.begin(21,22,400000);
  IMU1.initialize();
  IMU1.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

  // Setup hardware timers:
  //leftMotorTimer = timerBegin(0, 2, true);
  //timerAttachInterrupt(leftMotorTimer, &leftTimerFunc, true);
  //timerAttachInterrupt(leftMotorTimer, &leftTimerFunc, true);
  //timerAlarmWrite(leftMotorTimer, 22000, true);
  //timerAlarmEnable(leftMotorTimer);
  
  pinMode(19, OUTPUT);
  pinMode(33, OUTPUT);
  lastTime = millis();
 
  pinMode(2, OUTPUT);
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);
  digitalWrite(33, HIGH);
  engine.init();
  leftStepper = engine.stepperConnectToPin(2);
  rightStepper = engine.stepperConnectToPin(19);
  leftStepper->setDirectionPin(15);
  rightStepper->setDirectionPin(33);
  leftStepper->setAutoEnable(true);

  leftStepper->setSpeed(700);  // the parameter is us/step !!!
  rightStepper->setSpeed(700);

  leftStepper->setAcceleration(10000);
  rightStepper->setAcceleration(10000);
  lastDirection = 1;
  previousVel = 2.0;
}


float deltaGyroAngle, accAngle;
float gXSum = 0.0;
int16_t gyrX, gyrY, gyrZ, aX, aY, aZ, oX, oY, oZ;

void computeGyroOffsets(int8_t N = 10) {
  for (int i = 0; i <= 100; i++){
    IMU1.getRotation(&oX, &oY, &oZ);
    gyroscopeOffsets[0] += oX;
    gyroscopeOffsets[1] += oY;
    gyroscopeOffsets[2] += oZ;
  }
  gyroscopeOffsets[0] /= (N);
  gyroscopeOffsets[1] /= N;
  gyroscopeOffsets[2] /= N;
}

/**
 * EMA Moving Average Low Pass filter implementation, thetaOld is last value, thetaPrimeFiltered is the 
 *  resulting value, dT is the differential time unit
 **/
void emaLowPass(float *accAngle, float &thetaPrimeFiltered, float thetaOld){
  IMU1.getMotion6(&gyrX, &gyrY, &gyrZ, &aX, &aY, &aZ);
  *accAngle = atan2f((float) aY, (float) aZ) * 180.0/(2.0*acos(0.0)) - 2;
  //thetaPrimeFiltered = ((float)((gyrY - gyroscopeOffsets[1])) / GYROSCOPE_S) * dT;
  thetaPrimeFiltered = ((float)((gyrY)) / GYROSCOPE_S) * dT;
  thetaPrimeFiltered = ((EMA_ALPHA * thetaOld) +  thetaPrimeFiltered * (1 - EMA_ALPHA));
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

void complementaryFilter(float *pitch, float *roll) {
      IMU1.getMotion6(&aX, &aY, &aZ, &gyrX, &gyrY, &gyrZ);
      *pitch += (float) (gyrX/GYROSCOPE_S) * dT;
      *roll += (float) (gyrY/GYROSCOPE_S) * dT;
      mag = abs(aX) + abs(aY) + abs(aZ);
      if (mag > 8192 && mag < 32768) {
        pitchAcc = atan2f((float)aY, (float)aZ) * 180 / M_PI;
        *pitch = *pitch * 0.96 + pitchAcc * 0.04;
        rollAcc = atan2f((float)aX, (float)aZ) * 180 / M_PI;
        *roll = *roll * 0.96 + rollAcc * 0.04;
        //Serial.println(*pitch);
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

double thet = 0.0;

double integral, proportional, derivative, error, lastError=0.0, setpoint = 0.0, output;

double maxOutput = 10;
double minOutput = -10;

double kp = 1.2;
double ki = 0.0;
double kd = 0.0;

void loop(){
  currentTime = millis();
  if (currentTime - lastTime > DT_MILLIS) {
      complementaryFilter(&pitch, &roll);
      error = pitch;
      proportional = (0.0 - error) * kp;
      integral += ki * error * dT;
      if (integral > maxOutput) 
        integral = maxOutput;
      else if (integral < minOutput) 
        integral = minOutput;
      derivative = kd * (error - lastError)/dT;
      if (output > maxOutput) 
        output = maxOutput;
      if (output < minOutput) 
        output = minOutput;
      output = proportional + integral + derivative;
      Serial.println(output);
     leftStepper->setSpeed((int) (60000/abs(output)));
      rightStepper->setSpeed((int)(60000/abs(output)));
      if (output > 0) {
        leftStepper->runForward();
        rightStepper->runForward();
      }
      else if (output < 0) {
        leftStepper->runBackward();
        rightStepper->runBackward();
      }
      else {
        
      }
      lastError = error;

      lastTime = currentTime;
  }

  

}