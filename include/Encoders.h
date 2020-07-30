#include "SensorArray.h"
#include <../src/consts/const.h>

class Encoders {
    public:
    Encoders(SensorArray sensor_arr);
    void handle_R_interrupt();
    void handle_L_interrupt();
    void turnAround();
    void turnAroundContinuous();
    void driveStraight();
    void driveStraightContinuous();
    void driveStraight_Sonar(int mode);
    bool rightTurn90();
    bool leftTurn90();
    bool rightTurn90_bin();
    bool collision_bin();
    void stop();
    void backup();
    void leftTurnUntilTape();
    volatile int countL;
    volatile int countR;
    SensorArray sensor_array;
};