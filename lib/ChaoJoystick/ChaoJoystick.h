#include <utility>
#include <string>
#include <vector>

#define JOY_UPPER 128
#define JOY_LOWER -128
class ChaoJoystick{
    private:
        double x0Dz, y0Dz, x1Dz, y1Dz;
        double joystickValues[4];
    public:
        ChaoJoystick();
        ChaoJoystick(double x0Deadzone, double y0Deadzone, double x1Deadzone, double y1Deadzone);
        void getAxisInput(double *joystickInput);
        void setDeadZone(double x, double y);
        void init();
};