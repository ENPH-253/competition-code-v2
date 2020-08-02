#include <../src/consts/const.h>
#include <../src/pinSetup/pinSetup.h>
#include <../src/sonar_logic/sonar_logic.h>
//#include "SensorArray.h"
#include "Pid.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
void motorPIDcontrol(int speed);
void pivot(int direction, int speed);
void motorStraight();
void motorStop();
void handle_R_interrupt();
void handle_L_interrupt();
void depositCans();

void funThing();
void openGate();
void closeGate();
void depositCan();

Pid pid = Pid(KP, KD);
SensorArray sensor_array = SensorArray();
Encoders encoders = Encoders(sensor_array);
Sonar_Logic sl = Sonar_Logic(&encoders);
int pivot_count = 0;

void setup()
{
  if (digitalRead(COLLISION_SENSOR) == HIGH)
  {
    funThing();
  }
  pinSetup();
  attachInterrupt(digitalPinToInterrupt(PHOTOINTERRUPTER_R), handle_R_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(PHOTOINTERRUPTER_L), handle_L_interrupt, FALLING);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  Serial.begin(9600);
  pwm_start(RIGHT_SERVO, SERVO_FREQ, PLATFORM_DOWN_R, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(LEFT_SERVO, SERVO_FREQ, PLATFORM_DOWN_L, RESOLUTION_10B_COMPARE_FORMAT);

  //uncomment for start sequence
  //grab bin and pivot to tape

  // while(true){
  //   int error = sensor_array.calculateError();
  //   display.setCursor(0,0);

  //   display.println(sensor_array.LFSensor[0]);
  //   display.println(sensor_array.LFSensor[1]);
  //   display.println(sensor_array.LFSensor[2]);
  //   display.println(sensor_array.LFSensor[3]);
  //   display.println(sensor_array.LFSensor[4]);
  //   display.println(sensor_array.digitalArr[5]);

  //   display.display();
  //   display.clearDisplay();

  // }

  // Starting sequence
  encoders.backup(5, 5);
  pivot(LEFT, 860);
  while (sensor_array.digitalArr[5] != 1)
  {
    int error = sensor_array.calculateError();
    pid.calculatePID(error);
    motorPIDcontrol(890);
  }
  pivot(RIGHT, PIVOT_SPEED);
}

void loop()
{

  display.clearDisplay();
  display.setCursor(0, 0);

  // follow tape
  int error = sensor_array.calculateError();

  display.println(sensor_array.LFSensor[0]);
  display.println(sensor_array.LFSensor[1]);
  display.println(sensor_array.LFSensor[2]);
  display.println(sensor_array.LFSensor[3]);
  display.println(sensor_array.LFSensor[4]);
  display.println(error);
  display.display();

  pid.calculatePID(error);
  motorPIDcontrol(BASE_SPEED);

  if (sensor_array.digitalArr[5] == 1)
  {

    pivot(RIGHT, PIVOT_SPEED);
  }
  if (sl.pollSonar() < SONAR_LIMIT)
  {
    if (sl.pollSonar() < SONAR_LIMIT)
    {
      if (sl.pollSonar() < SONAR_LIMIT_CLOSE)
      {
        display.println("CLOSE");
        display.display();
        //small backwards movement
        encoders.adjustmentBackup(8);
        //open gate and small pivot
        openGate();
        encoders.rightPivotCount(16);

        //drive straight and close
        encoders.drive(18, 18, display);
        closeGate();
        //deposit
        depositCans();

        //pivot left
        encoders.backup(18, 18);
        pivot(LEFT, PIVOT_SPEED);
      }
      else if (sl.pollSonar() < SONAR_LIMIT_MID)
      {
        display.println("MMMMMM");
        display.display();
        //small backwards movement
        encoders.adjustmentBackup(11);
        //open gate and small pivot
        openGate();
        encoders.rightPivotCount(19);

        //drive straight and close
        encoders.drive(35, 35, display);
        closeGate();
        //deposit
        depositCans();

        //pivot left
        encoders.backup(37, 37);
        pivot(LEFT, PIVOT_SPEED);
      }

      else if (sl.pollSonar() < SONAR_LIMIT)
      {
        display.println("FFFFFFFFFFFFFFFFF");
        display.display();
        //small backwards movement
        encoders.adjustmentBackup(11);
        //open gate and small pivot
        openGate();
        encoders.rightPivotCount(26);

        //drive straight and close
        encoders.drive(57, 57, display);

        closeGate();

        //deposit
        depositCans();

        //pivot left
        encoders.backup(57, 57);
        pivot(LEFT, PIVOT_SPEED);
      }
    }
  }
}

void motorPIDcontrol(int speed)
{
  int gain = pid.speed;
  // int slow_down = pid.slow_down;

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
  pwm_start(MOTOR_R_F, MOTOR_FREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

  pwm_start(MOTOR_L_F, MOTOR_FREQ, 900, RESOLUTION_10B_COMPARE_FORMAT);
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
      pwm_start(MOTOR_R_F, MOTOR_FREQ, speed - 50, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

      pwm_start(MOTOR_L_B, MOTOR_FREQ, speed - 50, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    }
    if (millis() - motor_start > 200 && direction == RIGHT)
    {
      pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_B, MOTOR_FREQ, speed - 50, RESOLUTION_10B_COMPARE_FORMAT);

      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_F, MOTOR_FREQ, speed - 50, RESOLUTION_10B_COMPARE_FORMAT);
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