#include <chaoStepper.h>
#include <arduino.h>

chaoStepper::chaoStepper(uint8_t step, uint8_t dir, uint8_t tindex,void (*tfunc)()) {
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

void chaoStepper::setDir() {
    if currentDirection
}

int main(void) {
    ;
}

