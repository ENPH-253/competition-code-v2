#include "SensorArray.h"
#include "const.h"

class Encoders
{
public:
    Encoders(SensorArray sensor_arr);
    void handle_R_interrupt();
    void handle_L_interrupt();
    void drive(int leftStop, int rightStop,int speedL,int speedR);
    void turnL(int leftStop, int rightStop);
    void turnR(int leftStop, int rightStop);
    void backup(int leftStop, int rightStop, int speedL,int speedR);
    void stop();
    void rightPivot();
    void rightPivotCount(int counts);
    void adjustmentBackup();
    void adjustmentBackupCount(int counts);
    volatile int countL;
    volatile int countR;
    SensorArray sensor_array;
};