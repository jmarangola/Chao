#include <math.h>
class PIDController {
    public:
        PIDController(float kp, float ki, float kd, int16_t minOutput, int16_t maxOutput, float dt);

        void resetAll();
        void setParameters(float kp, float ki, float kd);
        void resetIntegral();

        void init();
        float compute();

        float outputMax, outputMin;
        float setpoint, input;

    private:
        float kp, kd, ki;

        float dt;
        float lastSetpoint, lastOutput, lastInput;
        float lastError, error;

        float proportional, integral, derivative;
};