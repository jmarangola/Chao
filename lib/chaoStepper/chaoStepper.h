#include <stdint.h>
#include <Arduino.h>
class chaoStepper {
        __int8_t stepPin, dirPin;
        hw_timer_t  * mTimer = NULL;
        int index;
        void (*func) ();
        void IRAM_ATTR pulseFunction();
        void init();
        void cycleUpdate();

        volatile bool stepState=0;
        volatile int step;
        volatile int8_t dir = 0;

        bool timerEnabled = 0;

        const float maxSpeed = 3000.0; 
        float lastVelocity = 0;

    public:
        chaoStepper(uint8_t step, uint8_t dir, uint8_t tindex,void (*tfunc)());

        float velocity = 0;


};