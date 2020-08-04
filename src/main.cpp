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
void goGetCan(int pivot_count, int drive_count, int backup_count);
void openGate();
void closeGate();
void funMode();

Pid pid = Pid(KP, KD);
SensorArray sensor_array = SensorArray();
Encoders encoders = Encoders(sensor_array);
Sonar_Logic sl = Sonar_Logic(&encoders);
int pivot_count = 0;
double start_time;
double check_time;
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

  // check tcrt and encoders

  // while (true) {
  // display.clearDisplay();
  // display.setCursor(0, 0);
  // //   // display.println(encoders.countL);
  // //   // display.println(encoders.countR);
  // int error = sensor_array.calculateError();
  //   display.println(sensor_array.LFSensor[0]);
  //   display.println(sensor_array.LFSensor[1]);
  //   display.println(sensor_array.LFSensor[2]);
  //   display.println(sensor_array.LFSensor[3]);
  //   display.println(sensor_array.LFSensor[4]);
  //   display.println(sensor_array.digitalArr[5]);

  //   display.display();

  // }

  if (!digitalRead(FUNSWITCH))
  {
    funMode();
  }
  start_time = millis();

  //grab bin and pivot to tape
  encoders.backup(85, 105, ENC_STRAIGHT_SPEED, ENC_STRAIGHT_SPEED + 35);
  pivot(LEFT, 880);
  // while (sensor_array.digitalArr[5] != 1)
  // {
  //   int error = sensor_array.calculateError();
  //   pid.calculatePID(error);
  //   motorPIDcontrol(900);
  // }
  // pivot(RIGHT, PIVOT_SPEED - 10);
}

void loop()
{
  // follow tape
  int error = sensor_array.calculateError();

  pid.calculatePID(error);
  motorPIDcontrol(BASE_SPEED);

  if (sensor_array.digitalArr[5] == 1)
  {
    // if (pivot_count == MAX_TURNS) {
    //   motorStop();
    //   while (true) {
    //   }
    // }
    pivot(RIGHT, BASE_SPEED - 10);
    // pivot_count++;
  }

  distance = sl.pollSonar();

  if(distance < SONAR_LIMIT)
  {
    if (distance < SONAR_LIMIT_CLOSE)
  {
    //small backwards movement
    encoders.adjustmentBackup();
    //open gate and small pivot
    openGate();
    encoders.rightPivotCount(14);

    //drive straight and close
    encoders.drive(30, 30, ENC_STRAIGHT_SPEED+10, ENC_STRAIGHT_SPEED);

    closeGate();
    //deposit

    depositCans();
    //pivot left
    encoders.backup(30, 30, ENC_STRAIGHT_SPEED+10, ENC_STRAIGHT_SPEED);
    pivot(LEFT, BASE_SPEED);
  }

  else if (distance < SONAR_LIMIT_MID)
  {
    //small backwards movement
    encoders.adjustmentBackup();
    //open gate and small pivot
    openGate();
    encoders.rightPivotCount(19);

    //drive straight and close
    encoders.drive(43, 43, ENC_STRAIGHT_SPEED+10, ENC_STRAIGHT_SPEED);

    closeGate();

    //deposit
    depositCans();

    //pivot left
    encoders.backup(43, 43, ENC_STRAIGHT_SPEED+10, ENC_STRAIGHT_SPEED);
    pivot(LEFT, BASE_SPEED);
  }

  else if (distance < SONAR_LIMIT)
  {
    //small backwards movement
    encoders.adjustmentBackup();
    //open gate and small pivot
    openGate();
    encoders.rightPivotCount(20);

    //drive straight and close
    encoders.drive(60, 60, ENC_STRAIGHT_SPEED+10, ENC_STRAIGHT_SPEED);

    closeGate();

    //deposit
    depositCans();

    //pivot left
    encoders.backup(60, 60, ENC_STRAIGHT_SPEED+10, ENC_STRAIGHT_SPEED);
    pivot(LEFT, BASE_SPEED);
  }
  }
}

void goGetCan(int pivot_count, int drive_count, int backup_count)
{
  //small backwards movement
  encoders.adjustmentBackupCount(backup_count);
  //open gate and small pivot
  openGate();
  encoders.rightPivotCount(pivot_count);

  //drive straight and close
  encoders.drive(drive_count, drive_count, ENC_STRAIGHT_SPEED, ENC_STRAIGHT_SPEED);

  closeGate();
  //deposit

  depositCans();
  //pivot left
  encoders.backup(drive_count, drive_count, ENC_STRAIGHT_SPEED, ENC_STRAIGHT_SPEED);
  pivot(LEFT, BASE_SPEED);
}

void motorPIDcontrol(int speed)
{
  int gain = pid.speed;
  // int slow_down = pid.slow_down;

  if (gain > 0)
  {
    // if (BASE_SPEED + gain > 1023)
    // {
    //   pwm_start(MOTOR_R_F, MOTOR_FREQ, speed, RESOLUTION_10B_COMPARE_FORMAT);
    //   pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    //   pwm_start(MOTOR_L_F, MOTOR_FREQ, speed - gain, RESOLUTION_10B_COMPARE_FORMAT);
    //   pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    // }

    pwm_start(MOTOR_R_F, MOTOR_FREQ, speed + gain, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    pwm_start(MOTOR_L_F, MOTOR_FREQ, speed, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    // pwm_start(MOTOR_L_F, MOTOR_FREQ, BASE_SPEED - slow_down, RESOLUTION_10B_COMPARE_FORMAT);
    // pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  }
  else
  {

    // if (BASE_SPEED - gain > 1023)
    // {
    //   pwm_start(MOTOR_R_F, MOTOR_FREQ, speed + gain, RESOLUTION_10B_COMPARE_FORMAT);
    //   pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    //   pwm_start(MOTOR_L_F, MOTOR_FREQ, speed, RESOLUTION_10B_COMPARE_FORMAT);
    //   pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    // }
    // pwm_start(MOTOR_R_F, MOTOR_FREQ, BASE_SPEED + slow_down, RESOLUTION_10B_COMPARE_FORMAT);
    // pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
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
      pwm_start(MOTOR_R_F, MOTOR_FREQ, speed - 70, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

      pwm_start(MOTOR_L_B, MOTOR_FREQ, speed - 70, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    }
    if (millis() - motor_start > 200 && direction == RIGHT)
    {
      pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_B, MOTOR_FREQ, speed - 70, RESOLUTION_10B_COMPARE_FORMAT);

      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_F, MOTOR_FREQ, speed - 70, RESOLUTION_10B_COMPARE_FORMAT);
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
  delay(500);
}

void closeGate()
{
  pwm_start(GATE_SERVO, SERVO_FREQ, GATE_CLOSED, RESOLUTION_10B_COMPARE_FORMAT);
  delay(500);
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
  int count = 8;
  while(count > 0){
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("FUNMODE!");
    display.println(count);
    display.display();
    delay(1000);
    count--;
  }
  //delaying a little bit extra
  delay(500);
  depositCans();
  display.clearDisplay();
  display.print("Depositing cans");
  display.display();
  
  while (true)
  {
    delay(1);
    //do nothing
  }
}