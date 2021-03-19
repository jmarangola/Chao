/**
 * Minimal PID library to control single and double inverted pendulum dynamics ESP32
 * 
 * Author: John Marangola
 * marangol@bc.edu
 * 
 * Project Github page:
 * https://github.com/jmarangola/Chao
 **/ 

#include "PIDController.h"
#include <iomanip>
#include <iostream>
#include <string>
#include <cmath>
using namespace std;

PIDController::PIDController(float kp, float ki, float kd, int16_t minOutput, int16_t maxOutput, float dt) {
    this->dt = dt;
    this->kp = kp;
    this->ki = ki*dt;
    this->kd = kd;
    outputMin = minOutput;
    outputMax = maxOutput;
    init();
}

void PIDController::init() {
    setpoint = 0.0;
    lastSetpoint = 0.0;
    lastInput = 0.0;
    lastError = 0.0;
    proportional = 0.0;
    derivative = 0.0;
    integral = 0.0;
}

float PIDController::compute() {
    float outputResult;
    error = setpoint - input;
    proportional = kp*error;
    
    integral += ki*error;
    if (integral > outputMax) 
        integral = outputMax;
    else if (integral < outputMin) 
        integral = outputMin;

    derivative = (error - lastError)*kd/dt;
    outputResult = proportional + integral - derivative;
    if (outputResult > outputMax) 
        outputResult = outputMax;
    else if (outputResult < outputMin)
        outputResult = outputMin;

    lastInput = input;
    return outputResult;
}

void PIDController::resetAll() {
    integral = 0.0;
    derivative = 0.0;
    proportional = 0.0;
    lastError = 0.0;
}

void PIDController::resetIntegral() {
    integral = 0.0;
}

void PIDController::setParameters(float kp, float ki, float kd) {
    this->dt = dt;
    this->kp = kp;
    this->ki = ki*dt;
}







