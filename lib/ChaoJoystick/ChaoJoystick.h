#include <utility>

class ChaoJoystick{
    private:
        double x0_dz, y0_dz, x1_dz, y1_dz;
    public:
        ChaoJoystick();
        ChaoJoystick(double x0Deadzone, double y0Deadzone, double x1Deadzone, double y1Deadzone);
        std::pair<double, double> getAxisOutput(int);
        void setDeadZone(double x, double y);
        std::pair<double, double> getDeadZone();
};