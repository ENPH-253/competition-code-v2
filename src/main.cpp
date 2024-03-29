#include "const.h"
#include "pinSetup.h"
#include "sonar_logic.h"
#include "Pid.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
void motorPIDcontrol(int speed);
void pivot(int direction, int speed);
void motorStraight();
void motorStop();
void handle_R_interrupt();
void handle_L_interrupt();
void depositCans();
void openGate();
void closeGate();
void funMode();

Pid pid = Pid(KP, KD);
SensorArray sensor_array = SensorArray();
Encoders encoders = Encoders(sensor_array);
Sonar_Logic sl = Sonar_Logic();
double start_time;
float distance;

void setup()
{
  pinSetup();
  attachInterrupt(digitalPinToInterrupt(PHOTOINTERRUPTER_R), handle_R_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(PHOTOINTERRUPTER_L), handle_L_interrupt, FALLING);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  pwm_start(RIGHT_SERVO, SERVO_FREQ, PLATFORM_DOWN_R, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(LEFT_SERVO, SERVO_FREQ, PLATFORM_DOWN_L, RESOLUTION_10B_COMPARE_FORMAT);

  if (!digitalRead(FUNSWITCH))
  {
    funMode();
  }
  start_time = millis();

  //grab bin and pivot to tape
  encoders.backup(65, 105, ENC_STRAIGHT_SPEED, ENC_STRAIGHT_SPEED + 60);
  pivot(LEFT, 860);
}

void loop()
{

  // follow tape
  int error = sensor_array.calculateError();

  // follow tape
  pid.calculatePID(error);
  motorPIDcontrol(BASE_SPEED);

  if (sensor_array.digitalArr[5] == 1)
  {
    pivot(RIGHT, PIVOT_SPEED);
  }

  distance = sl.pollSonar();

  if (distance < SONAR_LIMIT)
  {
    if (distance < SONAR_LIMIT_CLOSE)
    {
      //small backwards movement
      encoders.adjustmentBackup();
      //open gate and small pivot
      openGate();
      encoders.rightPivotCount(16);

      //drive straight and close
      encoders.drive(18, 18, ENC_STRAIGHT_SPEED, ENC_STRAIGHT_SPEED);
      closeGate();
      //deposit
      depositCans();

      //pivot left
      encoders.backup(17, 17, ENC_STRAIGHT_SPEED, ENC_STRAIGHT_SPEED);
      pivot(LEFT, PIVOT_SPEED);
    }
    else if (distance < SONAR_LIMIT_MID)
    {
      //small backwards movement
      encoders.adjustmentBackup();
      //open gate and small pivot
      openGate();
      encoders.rightPivotCount(19);

      //drive straight and close
      encoders.drive(35, 35, ENC_STRAIGHT_SPEED, ENC_STRAIGHT_SPEED);
      closeGate();
      //deposit
      depositCans();

      //pivot left
      encoders.backup(37, 37, ENC_STRAIGHT_SPEED, ENC_STRAIGHT_SPEED);
      pivot(LEFT, PIVOT_SPEED);
    }

    else if (distance < SONAR_LIMIT)
    {
      //small backwards movement
      encoders.adjustmentBackup();
      //open gate and small pivot
      openGate();
      encoders.rightPivotCount(22);

      //drive straight and close
      encoders.drive(60, 60, ENC_STRAIGHT_SPEED, ENC_STRAIGHT_SPEED);

      closeGate();

      //deposit
      depositCans();

      //pivot left
      encoders.backup(57, 57, ENC_STRAIGHT_SPEED, ENC_STRAIGHT_SPEED);
      pivot(LEFT, PIVOT_SPEED);
    }
  }
  if (millis() - start_time > FINAL_DUMP) {
    depositCans();
    motorStop();
    while(true) {

    }
  }
}

void motorPIDcontrol(int speed)
{
  int gain = pid.speed;

  if (gain > 0)
  {
    pwm_start(MOTOR_R_F, MOTOR_FREQ, speed + gain, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    pwm_start(MOTOR_L_F, MOTOR_FREQ, speed, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  }
  else
  {
    pwm_start(MOTOR_R_F, MOTOR_FREQ, speed, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    pwm_start(MOTOR_L_F, MOTOR_FREQ, speed - gain, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  }
}

void motorStop()
{
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
}

void motorStraight()
{
  pwm_start(MOTOR_R_F, MOTOR_FREQ, BASE_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
 
  pwm_start(MOTOR_L_F, MOTOR_FREQ, BASE_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
}

void pivot(int direction, int speed)
{
  int motor_start = millis();
  if (direction == LEFT)
  {
    pwm_start(MOTOR_R_F, MOTOR_FREQ, speed, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    pwm_start(MOTOR_L_B, MOTOR_FREQ, speed, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  }
  else if (direction == RIGHT)
  {
    pwm_start(MOTOR_L_F, MOTOR_FREQ, speed, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, speed, RESOLUTION_10B_COMPARE_FORMAT);
  }
  while (true)
  {

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("pivoting");

    sensor_array.calculateError();

    display.display();

    if (sensor_array.anyFrontSensorOn())
    {
      motorStop();
      break;
    }

    if (millis() - motor_start > 200 && direction == LEFT)
    {
      pwm_start(MOTOR_R_F, MOTOR_FREQ, speed - 80, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

      pwm_start(MOTOR_L_B, MOTOR_FREQ, speed - 80, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    }
    if (millis() - motor_start > 200 && direction == RIGHT)
    {
      pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_B, MOTOR_FREQ, speed - 65, RESOLUTION_10B_COMPARE_FORMAT);

      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_F, MOTOR_FREQ, speed - 65, RESOLUTION_10B_COMPARE_FORMAT);
    }
  }
  display.clearDisplay();
}

void handle_R_interrupt()
{
  encoders.handle_R_interrupt();
}

void handle_L_interrupt()
{
  encoders.handle_L_interrupt();
}

void openGate()
{
  pwm_start(GATE_SERVO, SERVO_FREQ, GATE_OPEN, RESOLUTION_10B_COMPARE_FORMAT);
}

void closeGate()
{
  pwm_start(GATE_SERVO, SERVO_FREQ, GATE_CLOSED, RESOLUTION_10B_COMPARE_FORMAT);
  delay(300);
}

void depositCans()
{
  pwm_start(LEFT_SERVO, SERVO_FREQ, PLATFORM_UP_L, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(RIGHT_SERVO, SERVO_FREQ, PLATFORM_UP_R, RESOLUTION_10B_COMPARE_FORMAT);

  delay(1000);

  pwm_start(LEFT_SERVO, SERVO_FREQ, PLATFORM_DOWN_L, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(RIGHT_SERVO, SERVO_FREQ, PLATFORM_DOWN_R, RESOLUTION_10B_COMPARE_FORMAT);

  delay(100);
}

void funMode()
{
  int count = 6;
  while (count > 0)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("FUNMODE!");
    display.print(count);
    display.display();
    count--;
    delay(1000);
  }

  depositCans();
  while (true)
  {
  }
}