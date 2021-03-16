#include "ChaoJoystick.h"
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <string>
#include <stdio.h>
#include <PS4Controller.h>
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

void ChaoJoystick::init(std::string macAddress="01:01:01:01:01:01") {
    PS4.begin(macAddress);
}

/**
 * Returns axis output with deadzone for a joystick index (0=left, 1=right).
 * Return : std::pair<double, double>, output
 **/
std::pair<double, double> ChaoJoystick::getAxisOutput(int axis) {
    double x_raw, y_raw;
    std::pair<double, double> axisOutput;
    if (PS4.isConnected()){
        if (axis == 0) {
            // Get axis raw values x_raw, y_raw here --
            //x_raw = PS4.data.analog.stick.lx;
            //y_raw = PS4.data.analog.stick.ly; 
            axisOutput = std::make_pair(((abs(x_raw - x0_dz) > 0) ? x_raw : 0.0),  ((abs(y_raw - y0_dz) > 0) ? y_raw : 0));
        }
        else if (axis == 1) {
            // Get axis raw values x_raw, y_raw here -- 
            x_raw = (double) PS4.data.analog.stick.rx;
            y_raw = (double) PS4.data.analog.stick.ry;
            axisOutput = std::make_pair(((abs(x_raw - x1_dz) > 0) ? x_raw : 0.0),  ((abs(y_raw - y1_dz) > 0) ? y_raw : 0));
        }
    }
    else {
        //std::cout << "Not connected" << std::endl;
        Serial.println("Not connected.");
        return axisOutput;
    }
    return axisOutput;
}

/**
 * 
 **/
int main(){
    
}