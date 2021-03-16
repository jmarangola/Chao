#include <math.h>
class PIDController {
    PIDController();
    void resetValues();
    void setParameters();
    void init();
    void update();
};