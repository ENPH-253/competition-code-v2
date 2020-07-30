#include "SensorArray.h"
#include <../src/consts/const.h>

class Encoders {
    public:
    Encoders(SensorArray sensor_arr);
    void handle_R_interrupt();
    void handle_L_interrupt();
    void drive(int rightStop, int leftStop);
    void turnL(int rightStop, int leftStop);
    void turnR(int rightStop, int leftStop);
    void stop();
    void backup();
    void rightPivot();
    volatile int countL;
    volatile int countR;
    SensorArray sensor_array;
};