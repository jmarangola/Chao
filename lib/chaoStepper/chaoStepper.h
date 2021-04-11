#include <stdint.h>
#include <Arduino.h>
class chaoStepper {
        __int8_t stepPin, dirPin;
        hw_timer_t  * mTimer = NULL;
        int index;
        void (*func) ();
        void init();


        volatile bool stepState=0;
        volatile int step;
        volatile int8_t dir = 0;

        bool timerEnabled = 0;

        const float maxSpeed = 3000.0; 
        float lastVelocity = 0;

    public:
        chaoStepper(uint8_t step, uint8_t dir, uint8_t tindex,void (*tfunc)());
        void IRAM_ATTR pulseFunction();
        void cycleUpdate();
        float velocity = 0;


};