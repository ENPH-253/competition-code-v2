#include "SensorArray.h"
#include <../src/consts/const.h>

class Encoders {
    public:
    Encoders(SensorArray sensor_arr);
    void handle_R_interrupt();
    void handle_L_interrupt();
    void drive(int leftStop, int rightStop);
    void turnL(int leftStop, int rightStop);
    void turnR(int leftStop, int rightStop);
    void stop();
    void backup();
    void rightPivot();
    void adjustmentBackup();
    volatile int countL;
    volatile int countR;
    SensorArray sensor_array;
};