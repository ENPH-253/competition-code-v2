#include <Arduino.h>
#include "Pid.h"

Pid::Pid(int kp, int kd)
{
  Kp = kp;
  Kd = kd;
  slow_ratio = 0.35;
  gain_ratio = 0.65;
}

void Pid::calculatePID(int error)
{
  p = error;
  d = error - previousError;
  speed = (Kp * p) + (Kd * d);
  previousError = error;
}
