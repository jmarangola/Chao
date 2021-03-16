#include <utility>
#include <string>
class ChaoJoystick{
    private:
        double x0_dz, y0_dz, x1_dz, y1_dz;
    public:
        ChaoJoystick();
        ChaoJoystick(double x0Deadzone, double y0Deadzone, double x1Deadzone, double y1Deadzone);
        ChaoJoystick(double x0Deadzone, double y0Deadzone, double x1Deadzone, double y1Deadzone, std::string macAddress);
        std::pair<double, double> getAxisOutput(int);
        void setDeadZone(double x, double y);
        void init(std::string macAddress);
        std::pair<double, double> getDeadZone();
};