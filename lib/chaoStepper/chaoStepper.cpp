#include <chaoStepper.h>
#include <Arduino.h>
#include <cmath>

chaoStepper::chaoStepper(uint8_t step, uint8_t dir, uint8_t tindex, void (*tfunc)()) {
    timerNumber = tindex;
    func = tfunc;
    this->step = step;
    this->dir = dir;
    init();
}

void chaoStepper::init() {
    pinMode(step, OUTPUT);
    pinMode(dir, OUTPUT);
}

void IRAM_ATTR fastStepper::pulseFunction() {
    if (!stepState) {
        _step += dir; 
        stepState = 1;
        GPIO.out_w1ts = 1<<_stepPin;
    } 
    else {
        stepState = 0;
        GPIO.out_w1tc = 1<<_stepPin;
    }
}

void fastStepper::update() {
    float speed;
    int8_t lastDir = dir;
    dir = (velocity > 0) ? 0 : 1;
    uint32_t timerSpeedInt;
    speed = abs(velocity);
    if (speed > maxSpeed) speed = maxSpeed; 

    if (dir != lastDir) // change direction if it differs from the old direction
        digitalWrite(_dirPin, ((dir == 1) ? 1 : 0));
    
    if (velocity != lastVelocity) {
        if (speed != 0) {
            timerSpeedInt = (uint32_t) (400000.0/absSpeed);
            timerAlarmWrite(_timer, timerSpeedInt, true);
            if (!timerEnabled) {
                timerAlarmEnable(_timer); // Re-enable timer
                timerEnabled = 1;
            } 
        }
        else {
            timerAlarmWrite(_timer, 100000, true);
            timerAlarmDisable(_timer);
            timerEnabled = 0;
        }
  }
  lastSpeed = speed;
}




