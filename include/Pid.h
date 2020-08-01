class Pid
{
public:
    Pid(int kp, int kd);
    void calculatePID(int error);
    int Kp, Kd;
    int p, d;
    // int error;
    int previousError;
    int speed;
    int slow_down;
    float slow_ratio;
    float gain_ratio;
};