#include <Arduino.h>
#include <../src/consts/const.h>
#include "Encoders.h"
#define MOTOR_R_CW PA_2
#define MOTOR_R_CCW PA_3

#define MOTOR_L_CW PB_8
#define MOTOR_L_CCW PB_9

#define PWMFREQ 2000

Encoders::Encoders(SensorArray sensor_arr) {
    countL = 0;
    countR = 0;
    sensor_array = sensor_arr;
}

void Encoders::handle_L_interrupt() {
    countL++;
}

void Encoders::handle_R_interrupt() {
    countR++;
}

void Encoders::drive(int leftStop, int rightStop) {
  delay(100);

  countL = 0;
  countR = 0;

  delay(100);

  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 850, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 850, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  bool rightDone = false;
  bool leftDone = false;

  while (!rightDone || !leftDone)
  {
    if (countR > rightStop && !rightDone) {
      pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      rightDone = true;
    }

    if (countL > leftStop && !leftDone) 
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      leftDone = true;
    }
  }
}

void Encoders::turnR(int leftStop, int rightStop) {
    delay(100);

    countL = 0;
    countR = 0;

    pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    bool rightDone = false;
    bool leftDone = false;

    while (!rightDone || !leftDone)
    {
      if (countR > rightStop && !rightDone) {
        pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        rightDone = true;
      }
      if (countL > leftStop && !leftDone) {
        pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        leftDone = true;
      }
    }
}

void Encoders::turnL(int leftStop, int rightStop) {
    delay(100);

    countL = 0;
    countR = 0;

    pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    bool rightDone = false;
    bool leftDone = false;

    while (!rightDone || !leftDone)
    {
      if (countR > rightStop && !rightDone) {
        pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        rightDone = true;
      }
      if (countL > leftStop && !leftDone) {
        pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        leftDone = true;
      }
    }
}

void Encoders::backup(int leftStop, int rightStop) {
  countL = 0;
  countR = 0;

  pwm_start(MOTOR_R_B, MOTOR_FREQ, 850, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 850, RESOLUTION_10B_COMPARE_FORMAT);

  bool rightDone = false;
  bool leftDone = false;

  while (!rightDone || !leftDone)
  {
    if (countR > rightStop && !rightDone) 
    {
      pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      rightDone = true;
    }

    if (countL > leftStop && !leftDone) 
    {
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      leftDone = true;
    }
  }
}

void Encoders::rightPivot() {
  turnR(0, 30);
}

void Encoders::adjustmentBackup() {
  backup(17, 17);
}