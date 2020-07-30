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

bool Encoders::rightTurn90() {
   delay(100);

    countL = 0;
    countR = 0;

    pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    bool rightDone = false;
    bool leftDone = false;

    // while(sensor_array.anyFrontSensorOn()){
    //   delay(500);
    // }

    while (!rightDone || !leftDone)
    {
        if (sensor_array.anyFrontSensorOn()) {
          this->stop();
          return false;
        }

        if (countR > ENC_R_FWD / 1.5 && !rightDone)  //46 41
        {
        pwm_start(MOTOR_R_CCW, PWMFREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_R_CW, PWMFREQ, 400, RESOLUTION_10B_COMPARE_FORMAT);
        if (!leftDone)
        {
            pwm_start(MOTOR_L_CW, PWMFREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
        }
        rightDone = true;
        }

        if (countL > ENC_L_FWD / 1.5 && !leftDone) //57 52
        {
        pwm_start(MOTOR_L_CW, PWMFREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_L_CCW, PWMFREQ, 400, RESOLUTION_10B_COMPARE_FORMAT);
        if (!rightDone)
        {
            pwm_start(MOTOR_R_CCW, PWMFREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
        }
        leftDone = true;
        }
    }

    return true;
}

bool Encoders::leftTurn90() {
   delay(100);

    countL = 0;
    countR = 0;

    pwm_start(MOTOR_R_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED-50, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, ENC_STRAIGHT_SPEED-50, RESOLUTION_10B_COMPARE_FORMAT);

    bool rightDone = false;
    bool leftDone = false;

    // while(sensor_array.anyFrontSensorOn()){
    //   delay(500);
    // }

    delay(750);

    while (!rightDone || !leftDone)
    {
        // if (sensor_array.anyFrontSensorOn()) {
          if (sensor_array.digitalArr[3] == 1 || sensor_array.digitalArr[4] == 1){
          delay(500);
          this->stop();
          return false;
        }

        if (countR > ENC_R_FWD / 2 + 100 && !rightDone)  //46 41
        {
        pwm_start(MOTOR_R_CCW, PWMFREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_R_CW, PWMFREQ, 400, RESOLUTION_10B_COMPARE_FORMAT);
        if (!leftDone)
        {
            pwm_start(MOTOR_L_CW, PWMFREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
        }
        rightDone = true;
        }

        if (countL > ENC_L_FWD / 2 + 100 && !leftDone) //57 52
        {
        pwm_start(MOTOR_L_CW, PWMFREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_L_CCW, PWMFREQ, 400, RESOLUTION_10B_COMPARE_FORMAT);
        if (!rightDone)
        {
            pwm_start(MOTOR_R_CCW, PWMFREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
        }
        leftDone = true;
        }
    }

    return true;
}

bool Encoders::rightTurn90_bin() {
  pwm_start(MOTOR_R_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

   delay(800);

    countL = 0;
    countR = 0;

    pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, ENC_STRAIGHT_SPEED-50, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED-50, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    bool rightDone = false;
    bool leftDone = false;

    while (!rightDone || !leftDone)
    {

        if (countR > ENC_R_FWD /1.5 && !rightDone)  //46 41
        {
        pwm_start(MOTOR_R_CCW, PWMFREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_R_CW, PWMFREQ, 400, RESOLUTION_10B_COMPARE_FORMAT);
        if (!leftDone)
        {
            pwm_start(MOTOR_L_CW, PWMFREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
        }
        rightDone = true;
        }

        if (countL > ENC_L_FWD / 1.5 && !leftDone) //57 52
        {
        pwm_start(MOTOR_L_CW, PWMFREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_L_CCW, PWMFREQ, 400, RESOLUTION_10B_COMPARE_FORMAT);
        if (!rightDone)
        {
            pwm_start(MOTOR_R_CCW, PWMFREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
        }
        leftDone = true;
        }
    }

    return true;
}

bool Encoders::collision_bin(){
  delay(100);

    countL = 0;
    countR = 0;

    pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, ENC_STRAIGHT_SPEED-50, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED-50, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    bool rightDone = false;
    bool leftDone = false;

    while (!rightDone || !leftDone)
    {
      if(digitalRead(COLLISION_SENSOR)){
        this->stop();
        return true;
      }

        if (countR > ENC_R_FWD /1.5 && !rightDone)  //46 41
        {
        pwm_start(MOTOR_R_CCW, PWMFREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        rightDone = true;
        }

        if (countL > ENC_L_FWD / 1.5 && !leftDone) //57 52
        {
        pwm_start(MOTOR_L_CW, PWMFREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        leftDone = true;
        }
    }

    while(!digitalRead(COLLISION_SENSOR)){
      
    }

    this->stop();
}


void Encoders::turnAround(){
    
    delay(100);

    countL = 0;
    countR = 0;

    this->stop();

    delay(100);

    pwm_start(MOTOR_R_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);

    bool rightDone = false;
    bool leftDone = false;

    while (!rightDone || !leftDone)
    {
        if (sensor_array.anyFrontSensorOn()) {
          this->stop();
          break;
        }

        if (countR > ENC_R_FWD && !rightDone)  //46 41
        {
        pwm_start(MOTOR_R_CCW, PWMFREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_R_CW, PWMFREQ, 400, RESOLUTION_10B_COMPARE_FORMAT);
        if (!leftDone)
        {
            pwm_start(MOTOR_L_CW, PWMFREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
        }
        rightDone = true;
        }

        if (countL > ENC_L_FWD && !leftDone) //57 52
        {
        pwm_start(MOTOR_L_CW, PWMFREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_L_CCW, PWMFREQ, 400, RESOLUTION_10B_COMPARE_FORMAT);
        if (!rightDone)
        {
            pwm_start(MOTOR_R_CCW, PWMFREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
        }
        leftDone = true;
        }
    }

}

void Encoders::turnAroundContinuous() {

    bool reachedTape = false;

    while (!reachedTape) {
      countL = 0;
      countR = 0;

      this->stop();

      pwm_start(MOTOR_R_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_B, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);

      // pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      // pwm_start(MOTOR_R_B, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
      // pwm_start(MOTOR_L_F, MOTOR_FREQ, ENC_STRAIGHT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
      // pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

      bool rightDone = false;
      bool leftDone = false;

      while (!rightDone || !leftDone)
      {
          if (sensor_array.anyFrontSensorOn()) {
            this->stop();
            reachedTape = true;
            break;
          }

          if (countR > ENC_R_FWD && !rightDone)  //46 41
          {
          pwm_start(MOTOR_R_CCW, PWMFREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
          pwm_start(MOTOR_R_CW, PWMFREQ, 400, RESOLUTION_10B_COMPARE_FORMAT);
          if (!leftDone)
          {
              pwm_start(MOTOR_L_CW, PWMFREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
          }
          rightDone = true;
          }

          if (countL > ENC_L_FWD && !leftDone) //57 52
          {
          pwm_start(MOTOR_L_CW, PWMFREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
          pwm_start(MOTOR_L_CCW, PWMFREQ, 400, RESOLUTION_10B_COMPARE_FORMAT);
          if (!rightDone)
          {
              pwm_start(MOTOR_R_CCW, PWMFREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
          }
          leftDone = true;
          }
      }
    }

}

void Encoders::driveStraight() {
  delay(100);

  countL = 0;
  countR = 0;

  this->stop();

  delay(100);

  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 850, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 850, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  bool rightDone = false;
  bool leftDone = false;

  while (!rightDone || !leftDone)
  {
    if (sensor_array.anyFrontSensorOn()) {
      this->stop();
      break;
    }

    if (countR > 14 && !rightDone)  //42
    {
      pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      if (!leftDone)
      {
        pwm_start(MOTOR_L_F, MOTOR_FREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
      }
      rightDone = true;
    }

    if (countL > 25 && !leftDone) //36
    {
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      if (!rightDone)
      {
        pwm_start(MOTOR_R_F, MOTOR_FREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
      }
      leftDone = true;
    }
  }
  }

  void Encoders::stop() {
    pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  }

  void Encoders::driveStraightContinuous() {
    bool reachedTape = false;
    while (!reachedTape) {

      countL = 0;
      countR = 0;

      pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_F, MOTOR_FREQ, 850, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 850, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

      bool rightDone = false;
      bool leftDone = false;

      while (!rightDone || !leftDone)
      {
        if (sensor_array.anyFrontSensorOn()) {
          this->stop();
          reachedTape = true;
          break;
        }

        if (countR > 14 && !rightDone)  //42
        {
          pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
          pwm_start(MOTOR_R_F, MOTOR_FREQ, 400, RESOLUTION_10B_COMPARE_FORMAT);
          if (!leftDone)
          {
            pwm_start(MOTOR_L_F, MOTOR_FREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
          }
          rightDone = true;
        }

        if (countL > 25 && !leftDone) //36
        {
          pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
          pwm_start(MOTOR_L_B, MOTOR_FREQ, 400, RESOLUTION_10B_COMPARE_FORMAT);
          if (!rightDone)
          {
            pwm_start(MOTOR_R_B, MOTOR_FREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
          }
          leftDone = true;
        }
      }
    }
  }

void Encoders::driveStraight_Sonar(int mode) {
  delay(100);

  countL = 0;
  countR = 0;

  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  delay(100);

  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 800, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 800, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  bool rightDone = false;
  bool leftDone = false;

  int threshR;
  int threshL;

  if(mode == 0){
    threshR = 13;
    threshL = 23;

  } else {
    threshR = 24;
    threshL = 42;

  }
  
  while (!rightDone || !leftDone)
  {

    if (countR >threshR && !rightDone)  //42
    {
      pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      if (!leftDone)
      {
        pwm_start(MOTOR_L_F, MOTOR_FREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
      }
      rightDone = true;
    }

    if (countL > threshL && !leftDone) //36
    {
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      if (!rightDone)
      {
        pwm_start(MOTOR_R_F, MOTOR_FREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
      }
      leftDone = true;
    }
  }
}

void Encoders::backup() {
  // delay(100);

  countL = 0;
  countR = 0;

  // this->stop();

  // delay(100);

  pwm_start(MOTOR_R_B, MOTOR_FREQ, 850, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 850, RESOLUTION_10B_COMPARE_FORMAT);

  bool rightDone = false;
  bool leftDone = false;

  while (!rightDone || !leftDone)
  {
    if (countR > 14 && !rightDone)  //42
    {
      pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      if (!leftDone)
      {
        pwm_start(MOTOR_L_F, MOTOR_FREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
      }
      rightDone = true;
    }

    if (countL > 25 && !leftDone) //36
    {
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      if (!rightDone)
      {
        pwm_start(MOTOR_R_F, MOTOR_FREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
      }
      leftDone = true;
    }
  }
}