/*#include "Wire.h" // For I2C communication
#include <Arduino.h>
#include <iomanip>
#include <string>
#include <math.h>


const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
double old_value, new_value = 0;
double filtered = 0.0;

// Complementary filter constants:
const double imu_dt = 5; // 5 ms sample rate
const double k_acc = 0.01;
const double k_gyro = 0.01;

// IMU variables:
double pitch, roll;

int16_t temperature; 
char tmp_str[7]; 

char * convert_int16_to_str(int16_t i) { // Converts int16 to string of uniform length
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

double get_runnning_avg_IMU_X(){
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false); 
    Wire.requestFrom(MPU_ADDR, 7*2, true); 
    gyro_x = Wire.read()<<8 | Wire.read();
    new_value = double(gyro_x);
    filtered = new_value * 0.01 + old_value * 0.99;
    //Serial.print(" | gZ = "); Serial.print(filtered);
    old_value = new_value;
    return filtered;
}

// Returns the average of an n-sample of imu x-axis data
double n_sample_IMU_X(int n){ 
    double sample_sum;
    for (int i = 0; i < n; i++)
        sample_sum += get_runnning_avg_IMU_X();
    return (sample_sum / n);
    
}

void setup() {
  Serial.begin(9600);
  Serial.println("here");
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begin transmission to the I2C slave (GY-521)
  Wire.write(0x6B);
  Wire.write(0); //  Wake up the MPU-6050
  Wire.endTransmission(true);  
}

void loop() {
  Serial.println(n_sample_IMU_X(50));
  delay(1);
}

*/
//#include <FastAccelStepper.h>
//#include <PS4Controller.h>
//using namespace std;

// As in StepperDemo for Motor 1 on AVR
//#define dirPinStepper    5
//#define enablePinStepper 6
//#define stepPinStepper   9  // OC1A in case of AVR



/*
#define stepPinStepper 21

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;


void setup() {
  // Initialize PS4 Controller:
  Serial.begin(9600);
  PS4.begin("03:03:03:03:03:03");
  Serial.println("Ready.");

  Serial.begin(9600);
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {

    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

    stepper->setSpeed(500);  // the parameter is us/step !!!
    stepper->setAcceleration(10000);
    //stepper->move(5000);
  }
}

void loop() {
  if ( PS4.event.analog_move.stick.lx ) { 
					Serial.print("Left Stick x at ");
					Serial.println(PS4.data.analog.stick.lx, DEC);
	}
  delay(3000);
}
*/
#include <Arduino.h>
#include <PS4Controller.h>

void setup()
{
    Serial.begin(9600);

    PS4.begin("01:01:01:01:01:01");
    Serial.println("Ready.");
	Serial.println("test");
}

void loop()
{
		// Below has all accessible outputs from the controller
    if(PS4.isConnected()) {
			if ( PS4.data.button.up )
					Serial.println("Up Button");
			if ( PS4.data.button.down )
					Serial.println("Down Button");
			if ( PS4.data.button.left )
					Serial.println("Left Button");
			if ( PS4.data.button.right )
					Serial.println("Right Button");
				
			if ( PS4.data.button.upright )
					Serial.println("Up Right");
			if ( PS4.data.button.upleft )
					Serial.println("Up Left");
			if ( PS4.data.button.downleft )
					Serial.println("Down Left");
			if ( PS4.data.button.downright )
					Serial.println("Down Right");
				
			if ( PS4.data.button.triangle )
					Serial.println("Triangle Button");
			if ( PS4.data.button.circle )
					Serial.println("Circle Button");
			if ( PS4.data.button.cross )
					Serial.println("Cross Button");
			if ( PS4.data.button.square )
					Serial.println("Square Button");
				
			if ( PS4.data.button.l1 )
					Serial.println("l1 Button");
			if ( PS4.data.button.r1 )
					Serial.println("r1 Button");
				
			if ( PS4.data.button.l3 )
					Serial.println("l3 Button");
			if ( PS4.data.button.r3 )
					Serial.println("r3 Button");
				
			if ( PS4.data.button.share )
					Serial.println("Share Button");
			if ( PS4.data.button.options )
					Serial.println("Options Button");
				
			if ( PS4.data.button.ps )
					Serial.println("PS Button");
			if ( PS4.data.button.touchpad )
					Serial.println("Touch Pad Button");
				
			if ( PS4.data.button.l2 ) {
					Serial.print("l2 button at ");
					Serial.println(PS4.data.analog.button.l2);
			}
			if ( PS4.data.button.r2 ) {
					Serial.print("r2 button at ");
					Serial.println(PS4.data.analog.button.r2, DEC);
			}

			if ( PS4.event.analog_move.stick.lx ) {
					Serial.print("Left Stick x at ");
					Serial.println(PS4.data.analog.stick.lx, DEC);
			}
			if ( PS4.event.analog_move.stick.ly ) {
					Serial.print("Left Stick y at ");
					Serial.println(PS4.data.analog.stick.ly, DEC);
			}
			if ( PS4.event.analog_move.stick.rx ) {
					Serial.print("Right Stick x at ");
					Serial.println(PS4.data.analog.stick.rx, DEC);
			}
			if ( PS4.event.analog_move.stick.ry ) {
					Serial.print("Right Stick y at ");
					Serial.println(PS4.data.analog.stick.ry, DEC);
			}

     if (PS4.data.status.charging)
        Serial.println("The controller is charging");
     if (PS4.data.status.audio)
        Serial.println("The controller has headphones attached");
     if (PS4.data.status.mic)
        Serial.println("The controller has a mic attached");

     Serial.print("Battey Percent : ");
     Serial.println(PS4.data.status.battery, DEC);

		 Serial.println();
		 // This delay is to make the Serial Print more human readable
		 // Remove it when you're not trying to see the output
     delay(10);
    }
}