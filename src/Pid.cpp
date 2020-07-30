#include <Arduino.h>
#include "Pid.h"

Pid::Pid(int kp) {
    Kp = kp;
    Kd = 100; 
    // p = P;
    // d = 0;
    // error = 0;
    // previousError = 0;
    slow_ratio = 0.5;
    gain_ratio = 0.5;
}

void Pid::calculatePID(int error) {
    p = error;
    d = error - previousError;
    speed = (Kp * p) + (Kd * d);
    speed *= gain_ratio;
    slow_down = speed / slow_ratio;
    previousError = error;
}
