#include <utility>

class ChaoJoystick{
    private:
        double xdz, ydz;
    public:
        ChaoJoystick();
        ChaoJoystick(double xDeadzone, double yDeadzone);
        std::pair<double, double> getAxisOutput(int);
        void setDeadZone(double x, double y);
        std::pair<double, double> getDeadZone();
};