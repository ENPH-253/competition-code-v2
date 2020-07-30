#include <../src/sonar_logic/sonar_logic.h>

Sonar_Logic::Sonar_Logic(Encoders * encoder){
    this->count = 0;
    this->backedUp = 0;
    this->pickedUp = false;
    this->encoder = encoder;

    resetPWM();
}

void Sonar_Logic::driveTillFour(){
  
  while (pollSonar() > 6)
  {
    pwm_start(MOTOR_L_B, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, PWM_FREQUENCY, 780, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, PWM_FREQUENCY, 780, RESOLUTION_10B_COMPARE_FORMAT);
  delay(100);
  }

  stopMotors();
  
  
}

void Sonar_Logic::goGetEm(){
    count = 0;
    backedUp = 0;
    pickedUp = false;

    stopMotors();

    while(!pickedUp){
    if (pollSonar() < SONAR_LIMIT - count * STRAIGHT_LENGTH)
    {
        // encoder->driveStraight_Sonar(0);
        // stopMotors();

        delay(500);

        count++;
    } else {
        while (pollSonar() > SONAR_LIMIT - count * STRAIGHT_LENGTH)
        {
            pivotRight();
            stopMotors();

            if (pollSonar() <= CAN_LIMIT)
            {
                break;
            }
        }
    }

    if (pollSonar() <= CAN_LIMIT)
    {
        approachCan();

        // while(true)
        //     wait();
        // }
    }
}
}


float Sonar_Logic::pollSonar()
{
  float duration;
  float distance;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

//   display.setCursor(0, 45);
//   display.println("sonar:");
//   display.println(distance);
//   display.display();

  return distance;
}

void Sonar_Logic::stopMotors()
{
  pwm_start(MOTOR_L_B, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);

//   display.clearDisplay();
//   display.setCursor(0, 0);
//   display.println("stopping motors");
//   display.display();

  delay(200);
}

void Sonar_Logic::driveStraight()
{
  pwm_start(MOTOR_L_B, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, PWM_FREQUENCY, STRAIGHT_L, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, PWM_FREQUENCY, STRAIGHT_R, RESOLUTION_10B_COMPARE_FORMAT);

//   display.clearDisplay();
//   display.setCursor(0, 0);
//   display.println("driving straight");
//   display.display();

  count++;

  delay(250);
}

void Sonar_Logic::pivotLeft()
{
  while(pollSonar() > SONAR_LIMIT){
    pwm_start(MOTOR_L_B, PWM_FREQUENCY, LEFT_TURN_L, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_F, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);

    delay(TURN_TIME / TURN_SCALE);
  }

  for(int i = 0; i < 3; i++){
    pwm_start(MOTOR_L_B, PWM_FREQUENCY, LEFT_TURN_L, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_F, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);

    delay(TURN_TIME / TURN_SCALE);
  }
}

void Sonar_Logic::approachCan()
{
  int distance = pollSonar();

  while (distance < CAN_TOO_CLOSE)
  {
    backedUp++;

    pwm_start(MOTOR_L_B, PWM_FREQUENCY, STRAIGHT_L+75, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, PWM_FREQUENCY, STRAIGHT_R, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_F, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);

    delay(175);

    stopMotors();

    delay(50);

    distance = pollSonar();
  
    while (distance > SONAR_LIMIT - count * STRAIGHT_LENGTH)
    {
      pivotRight();
      stopMotors();

      if (pollSonar() <= CAN_LIMIT)
      {
        break;
      }
    }
  }

  pwm_start(GATE_SERVO, SERVO_FREQ, GATE_OPEN, RESOLUTION_10B_COMPARE_FORMAT);

  for(int i = 0; i< 7; i++){
    pivotRight();
    stopMotors();
  }
  
  delay(1500);

  // encoder->driveStraight_Sonar(1);

  stopMotors();

  delay(500);

  pwm_start(GATE_SERVO, SERVO_FREQ, GATE_CLOSED, RESOLUTION_10B_COMPARE_FORMAT);

  this->pickedUp = true;
  
  delay(2000);
}

void Sonar_Logic::pivotRight()
{
  pwm_start(MOTOR_L_B, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_F, PWM_FREQUENCY, LEFT_TURN_L, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, PWM_FREQUENCY, 600, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_F, PWM_FREQUENCY, STOP, RESOLUTION_10B_COMPARE_FORMAT);

//   display.clearDisplay();
//   display.setCursor(0, 0);
//   display.println("pivoting right");
//   display.display();

  delay(TURN_TIME / TURN_SCALE);
}

void Sonar_Logic::lowerPlat()
{
  // pwm_start(LEFT_SERVO, SERVO_FREQ, PLATFORM_DOWN_L, RESOLUTION_10B_COMPARE_FORMAT);
  // pwm_start(RIGHT_SERVO, SERVO_FREQ, PLATFORM_UP_R, RESOLUTION_10B_COMPARE_FORMAT);

//   display.clearDisplay();
//   display.setCursor(0, 0);
//   display.println("lowering platform");
//   display.display();

  delay(2000);
}

void Sonar_Logic::wait()
{
    // display.clearDisplay();
    // display.setCursor(0, 0);
    // display.println("straight chillin");
    // display.display();

    delay(3000);
}

int Sonar_Logic::angleToPWM(float angle)
{
  float pwm = map(angle, 0, 180, 2.7, 12);
  float input = pwm * 10.23;

//   display.clearDisplay();
//   display.setCursor(0, 0);
//   display.println((int)input);
//   display.display();

  delay(1000);

  return (int) input;
}

void Sonar_Logic::resetPWM()
{
  // setting gate closed, platform down and motors initially stopped
  pwm_start(GATE_SERVO, SERVO_FREQ, GATE_CLOSED, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(PB_1, SERVO_FREQ, PLATFORM_DOWN_L, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(PB_0, SERVO_FREQ, PLATFORM_DOWN_R, RESOLUTION_10B_COMPARE_FORMAT);
  stopMotors();

  delay(500);
}