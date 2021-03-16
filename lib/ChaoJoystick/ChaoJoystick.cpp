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
ChaoJoystick::ChaoJoystick(double x0Deadzone, double y0Deadzone, double x1Deadzone, double y1Deadzone) : x0_dz(x0Deadzone), y0_dz(y0Deadzone), x1_dz(x1Deadzone), y1_dz(y1Deadzone) {
    init();// initialize PS4 Controller
}

void ChaoJoystick::init() {
    PS4.begin("01:01:01:01:01:01");
}

/**
 * Returns axis output with deadzone for a joystick
 * Return : std::pair<double, double>, output
 **/
std::vector<double> ChaoJoystick::getAxisOutput() {
    double xRaw0=this->joystickValues[0], yRaw0=this->joystickValues[1], xRaw1=this->joystickValues[2], yRaw1=this->joystickValues[3];
    std::vector<double> axisOutput(2);
    if (PS4.isConnected()){
        if (PS4.event.analog_move.stick.lx) 
            xRaw0 = (double) PS4.data.analog.stick.lx;
        if (PS4.event.analog_move.stick.ly) 
            yRaw0 = (double) PS4.data.analog.stick.lx;
        if (PS4.event.analog_move.stick.rx) 
            xRaw1 = (double) PS4.data.analog.stick.rx;
        if (PS4.event.analog_move.stick.ry) 
            yRaw1 = (double) PS4.data.analog.stick.ry; 
        axisOutput.push_back(((abs(xRaw0 - x0_dz) > 0) ? xRaw0 : 0.0));
        axisOutput.push_back(((abs(yRaw0 - y0_dz) > 0) ? yRaw0 : 0.0));
        axisOutput.push_back(((abs(xRaw1 - x1_dz) > 0) ? xRaw1 : 0.0));
        axisOutput.push_back(((abs(yRaw1 - y1_dz) > 0) ? yRaw1 : 0.0));
    }
    else {
        Serial.println("Not connected.");
        return axisOutput;
    }
    this->joystickValues = axisOutput;
    return axisOutput;
}
