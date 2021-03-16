#include "ChaoJoystick.h"
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <string>
#include <stdio.h>
#include <PS4Controller.h>
#include <vector>
//#include "../PS4Controller.h"

/**
*   Default Constructor  
**/
ChaoJoystick::ChaoJoystick() {}

/**
*   Standard constructor takes two arguments for x, y deadzone constants
**/
ChaoJoystick::ChaoJoystick(double x0Deadzone, double y0Deadzone, double x1Deadzone, double y1Deadzone) : x0Dz(x0Deadzone), y0Dz(y0Deadzone), x1Dz(x1Deadzone), y1Dz(y1Deadzone) {
    init();// initialize PS4 Controller
}

void ChaoJoystick::init() {
    PS4.begin("01:01:01:01:01:01");
}

/**
 * Returns axis output with deadzone for a joystick
 * Return : std::pair<double, double>, output
 **/
void ChaoJoystick::getAxisInput(double *joystickInput) {
    double xRaw0=this->joystickValues[0], yRaw0=this->joystickValues[1], xRaw1=this->joystickValues[2], yRaw1=this->joystickValues[3];
    double normalizedInput[4];
    if (PS4.isConnected()){
        if (PS4.event.analog_move.stick.lx) {
            xRaw0 = (double) PS4.data.analog.stick.lx;
            normalizedInput[0] = (abs(xRaw0 - x0Dz) > 0) ? xRaw0 : 0.0;
        }
        if (PS4.event.analog_move.stick.ly) {
            yRaw0 = (double) PS4.data.analog.stick.lx;
            normalizedInput[1] = (abs(yRaw0 - y0Dz) > 0) ? yRaw0 : 0.0;
        }
        if (PS4.event.analog_move.stick.rx) {
            xRaw1 = (double) PS4.data.analog.stick.rx;
            normalizedInput[2] = (abs(xRaw1 - x1Dz) > 0) ? xRaw1 : 0.0;
        }
        if (PS4.event.analog_move.stick.ry) {
            yRaw1 = (double) PS4.data.analog.stick.ry; 
            normalizedInput[3] = (abs(yRaw1 - y1Dz) > 0) ? yRaw1 : 0.0;
        }
        // Normalize joystick inputs to percentage of maximum velocity by X: Z -> R [-1, 1]
        for (int i = 0; i < 4; i++){
            if (normalizedInput[i] == 0)
                continue;
            normalizedInput[i] = (normalizedInput[i] > 0) ? (JOY_UPPER - normalizedInput[i])/JOY_UPPER : (JOY_LOWER - normalizedInput[i])/JOY_LOWER;
        }
    }
    else {
        Serial.println("Not connected.");
    }
    joystickInput = normalizedInput;
    //joystickValues = joystickInput;
}
