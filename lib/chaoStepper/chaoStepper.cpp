#include <chaoStepper.h>
#include <Arduino.h>
#include <cmath>

chaoStepper::chaoStepper(uint8_t step, uint8_t dir, uint8_t tindex, void (*tfunc)()) {
    func = tfunc;
    index = tindex;
    stepPin = step;
    dirPin = dir;
    init();
}

void chaoStepper::init() {
    pinMode(step, OUTPUT);
    pinMode(dir, OUTPUT);
    mTimer = timerBegin(index, 2, true);
    timerAttachInterrupt(mTimer, func, true);
    timerAlarmEnable(mTimer);
}

void IRAM_ATTR chaoStepper::pulseFunction() {
    if (!stepState) {
        //_step += dir; 
        Serial.println(stepState);
        stepState = 1;
        GPIO.out_w1ts = 1<< stepPin;
        //Serial.println("here");
    } 
    else {
        Serial.println(stepState);
        stepState = 0;
        GPIO.out_w1tc = 1<< stepPin;
    }
}

void chaoStepper::cycleUpdate() {
    float speed;
    int8_t lastDir = dir;
    dir = (velocity > 0) ? 0 : 1;
    uint32_t timerSpeedInt;
    speed = abs(velocity);
    if (speed > maxSpeed) speed = maxSpeed; 

    if (dir != lastDir) // change direction if it differs from the old direction
        digitalWrite(dirPin, ((dir == 1) ? 1 : 0));
    
    if (velocity != lastVelocity) {
        if (speed != 0) {
            timerSpeedInt = (uint32_t) (500);
            timerAlarmWrite(mTimer, timerSpeedInt, true);
            if (!timerEnabled) {
                timerAlarmEnable(mTimer); // Re-enable timer
                timerEnabled = 1;
            } 
        }
        else {
            timerAlarmWrite(mTimer, 100000, true);
            timerAlarmDisable(mTimer);
            timerEnabled = 0;
        }
    }
  lastVelocity = velocity;
}




