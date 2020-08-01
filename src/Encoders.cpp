#include <Arduino.h>
#include <../src/consts/const.h>
#include "Encoders.h"
#define MOTOR_R_CW PA_2
#define MOTOR_R_CCW PA_3

#define MOTOR_L_CW PB_8
#define MOTOR_L_CCW PB_9

#define PWMFREQ 2000
#define FWD_SPEED 950
#define BACK_SPEED 950
// #define PIVOT_SPEED 950

Encoders::Encoders(SensorArray sensor_arr)
{
  countL = 0;
  countR = 0;
  sensor_array = sensor_arr;
}

void Encoders::handle_L_interrupt()
{
  countL++;
}

void Encoders::handle_R_interrupt()
{
  countR++;
}

void Encoders::drive(int leftStop, int rightStop, Adafruit_SSD1306 display)
{

  countL = 0;
  countR = 0;

  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, MOTOR_FREQ, FWD_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, FWD_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  delay(100);

  bool rightDone = false;
  bool leftDone = false;

  while (!rightDone || !leftDone)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(countL);
    display.println(countR);
    display.display();
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

void Encoders::turnR(int leftStop, int rightStop)
{

  countL = 0;
  countR = 0;

  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, PIVOT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, PIVOT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  delay(100);

  bool rightDone = false;
  bool leftDone = false;

  while (!rightDone || !leftDone)
  {
    if (countR > rightStop && !rightDone)
    {
      pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      rightDone = true;
      delay(100);
    }
    if (countL > leftStop && !leftDone)
    {
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      leftDone = true;
      delay(100);
    }
  }
}

void Encoders::turnL(int leftStop, int rightStop)
{

  countL = 0;
  countR = 0;

  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, MOTOR_FREQ, PIVOT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, PIVOT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  delay(100);

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

void Encoders::backup(int leftStop, int rightStop)
{
  countL = 0;
  countR = 0;

  pwm_start(MOTOR_R_B, MOTOR_FREQ, BACK_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, BACK_SPEED, RESOLUTION_10B_COMPARE_FORMAT);

  delay(100);

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

void Encoders::rightPivot()
{
  turnR(0, 23);
}

void Encoders::adjustmentBackup(int counts)
{
  backup(counts, counts);
}
void Encoders::rightPivotCount(int counts)
{
  turnR(0, counts);
}
