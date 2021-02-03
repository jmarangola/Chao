#include "FastAccelStepper.h"

#define dirPinStepper 18
#define enablePinStepper 26
#define stepPinStepper 22

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper);
    //stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);
    stepper->setSpeed(500);  // the parameter is us/step !!!
    stepper->setAcceleration(1000);
    //stepper->move(100000);
  }
}

void loop() {
  stepper->move(200);
  delay(15);
}