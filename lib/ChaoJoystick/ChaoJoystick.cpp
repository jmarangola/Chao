#include "ChaoJoystick.h"
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <string>
#include <stdio.h>
//#include "../PS4Controller.h"

/**
*   Default Constructor  
**/
ChaoJoystick::ChaoJoystick() {}

/**
*   Standard constructor takes two arguments for x, y deadzone constants
**/
ChaoJoystick::ChaoJoystick(double x0Deadzone, double y0Deadzone, double x1Deadzone, double y1Deadzone) : x0_dz(x0Deadzone), y0_dz(y0Deadzone), x1_dz(x1Deadzone), y1_dz(y1Deadzone) {
    ;// initialize PS4 Controller
}

/**
 * Returns axis output with deadzone for a joystick index (0=left, 1=right).
 * Return : std::pair<double, double>, output
 **/
std::pair<double, double> ChaoJoystick::getAxisOutput(int axis) {
    double x_raw, y_raw;
    std::pair<double, double> axisOutput;
    if (axis == 0) {
        // Get axis raw values x_raw, y_raw here -- 
        axisOutput = std::make_pair(((abs(x_raw - x0_dz) > 0) ? x_raw : 0.0),  ((abs(y_raw - y0_dz) > 0) ? y_raw : 0));
    }
    else if (axis == 0) {
        // Get axis raw values x_raw, y_raw here -- 
        axisOutput = std::make_pair(((abs(x_raw - x1_dz) > 0) ? x_raw : 0.0),  ((abs(y_raw - y1_dz) > 0) ? y_raw : 0));
    }
    else {
        std::cout << "Usage: expected <int> axis 0 or 1" << endl;
        return axisOutput;
    }
    return axisOutput;
}

/**
 * 
 **/
int main(){
    
}