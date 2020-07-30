#include <../src/consts/const.h>
#include <../include/Encoders.h>

class Sonar_Logic {
    public:
        Sonar_Logic(Encoders* encoder);
        void goGetEm();
        float pollSonar();
        void stopMotors();
        void pivotLeft();
        void pivotRight();
        void resetPWM();
        void driveTillFour();

    private:
        int count;
        int backedUp;
        bool pickedUp;
        Encoders * encoder;
        
        void driveStraight();
        
        void approachCan();
        void depositCan();
        void wait();
        void spin();
        void lowerPlat();
        
        int angleToPWM(float angle);
};