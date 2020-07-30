#include <Arduino.h>
#include "Pid.h"

Pid::Pid(int kp) {
    Kp = kp;
    Kd =  0; 
    slow_ratio = 0.35;
    gain_ratio = 0.65;
}

void Pid::calculatePID(int error) {
    p = error;
    d = error - previousError;
    speed = (Kp * p) + (Kd * d);
  //  speed *= gain_ratio;
    //slow_down = speed / slow_ratio;
    previousError = error;
}
