#include <stdint.h>

class chaoStepper {
        __int8_t step, dir, timerNumber;
        void (*func) ();
        void init();
    public:
        chaoStepper(uint8_t step, uint8_t dir, uint8_t tindex,void (*tfunc)());


};