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
ChaoJoystick::ChaoJoystick(double xDeadZone, double yDeadZone) : xdz(xDeadZone), ydz(yDeadZone) {
    this->xdz = xDeadZone;
    this->ydz = yDeadZone;
}

/**
 * Returns axis output with deadzone for a joystick index (0=left, 1=right).
 * Return : std::pair<double, double>, output
 **/
std::pair<double, double> ChaoJoystick::getAxisOutput(int axis) {
    double x_raw, x_v;
    double y_raw, y_v;
    std::pair<double, double> axisOutput( ((abs(x_raw - xdz) > 0) ? x_raw : 0.0),  ((abs(y_raw - ydz) > 0) ? y_raw : 0) );
    return axisOutput;
}

/**
 * 
 **/
std::pair<double, double> ChaoJoystick::getDeadZone(){
    std::pair<double, double> output(this->xdz, this->ydz);
    return output;
}

/**
 * 
 **/
int main(){
    ChaoJoystick joy(10, 10);
    printf("test\n");
}