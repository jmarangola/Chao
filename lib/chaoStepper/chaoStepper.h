#include <stdint.h>

class chaoStepper {
        __int8_t step, dir, timerNumber;
        void (*func) ();

        enum direction {FORWARD, BACKWARD};
        direction currentDirection;

        void init();
        void setDir(direction);

    public:
        chaoStepper(uint8_t step, uint8_t dir, uint8_t tindex,void (*tfunc)());


};