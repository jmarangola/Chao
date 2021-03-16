#include <utility>
#include <string>
#include <vector>
class ChaoJoystick{
    private:
        double x0_dz, y0_dz, x1_dz, y1_dz;
        std::vector<double> joystickValues;
    public:
        ChaoJoystick();
        ChaoJoystick(double x0Deadzone, double y0Deadzone, double x1Deadzone, double y1Deadzone);
        std::vector<double> getAxisOutput();
        void setDeadZone(double x, double y);
        void init();

};