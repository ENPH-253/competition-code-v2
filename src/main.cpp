#include <../src/consts/const.h>
#include <../src/pinSetup/pinSetup.h>
#include <../src/sonar_logic/sonar_logic.h>
//#include "SensorArray.h"
#include "Pid.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
void motorPIDcontrol();
void pivot(int direction);
void motorStraight();
void motorStop();
void handle_R_interrupt();
void handle_L_interrupt();
void depositCans();

void openGate();
void closeGate();
void depositCan();
void funMode();

Pid pid = Pid(KP, KD);
SensorArray sensor_array = SensorArray();
Encoders encoders = Encoders(sensor_array);
Sonar_Logic sl = Sonar_Logic(&encoders);
int pivot_count = 0;

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

  // Testing TCRTS in beginning

  // while(true){

  //   display.clearDisplay();
  //   display.setCursor(0, 0);

  //   display.println(encoders.countL);
  //   display.println(encoders.countR);
  //   int error = sensor_array.calculateError();
  //   display.println(sensor_array.LFSensor[0]);
  //   display.println(sensor_array.LFSensor[1]);
  //   display.println(sensor_array.LFSensor[2]);
  //   display.println(sensor_array.LFSensor[3]);
  //   display.println(sensor_array.LFSensor[4]);
  //   display.println(sensor_array.digitalArr[5]);

  //   display.display();
  // }

  // Reseting platform to down position
  pwm_start(RIGHT_SERVO, SERVO_FREQ, PLATFORM_DOWN_R, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(LEFT_SERVO, SERVO_FREQ, PLATFORM_DOWN_L, RESOLUTION_10B_COMPARE_FORMAT);

  if (!digitalRead(FUNSWITCH))
  {
    funMode();
  }

  //grab bin and pivot to tape
  encoders.backup(4, 4);
  pivot(LEFT);
}

void loop()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(sensor_array.LFSensor[0]);
  display.println(sensor_array.LFSensor[1]);
  display.println(sensor_array.LFSensor[2]);
  display.println(sensor_array.LFSensor[3]);
  display.println(sensor_array.LFSensor[4]);

  int error = sensor_array.calculateError();

  display.println(error);
  display.display();

  // int g = map(analogRead(POT), 0, 1023, 0, 150);
  // pid.Kp = g;
  // display.println(pid.Kp);
  // display.println(" ");

  // follow tape
  pid.calculatePID(error);
  motorPIDcontrol();

  if (sensor_array.digitalArr[5] == 1)
  {
    pivot(RIGHT);
  }

  if (sl.pollSonar() < SONAR_LIMIT_CLOSE)
  {
    //small backwards movement
    encoders.adjustmentBackup();
    //open gate and small pivot
    openGate();
    encoders.rightPivotCount(14);

    //drive straight and close
    encoders.drive(24, 24);

    closeGate();
    //deposit

    depositCans();
    //pivot left
    encoders.backup(24, 24);
    pivot(LEFT);
  }
  else if (sl.pollSonar() < SONAR_LIMIT_MID)
  {

    //small backwards movement
    encoders.adjustmentBackup();
    //open gate and small pivot
    openGate();
    encoders.rightPivotCount(18);

    //drive straight and close
    encoders.drive(43, 43);

    closeGate();

    //deposit
    depositCans();

    if (!digitalRead(FUNSWITCH))
    {
      funMode();
    }

    //pivot left
    encoders.backup(43, 43);
    pivot(LEFT);
  }

  else if (sl.pollSonar() < SONAR_LIMIT)
  {
    //small backwards movement
    encoders.adjustmentBackup();
    //open gate and small pivot
    openGate();
    encoders.rightPivotCount(23);

    //drive straight and close
    encoders.drive(62, 62);

    closeGate();

    //deposit
    depositCans();

    //pivot left
    encoders.backup(62, 62);
    pivot(LEFT);
  }
}

void motorPIDcontrol()
{
  int gain = pid.speed;
  // int slow_down = pid.slow_down;

  if (gain > 0)
  {
    pwm_start(MOTOR_R_F, MOTOR_FREQ, BASE_SPEED + gain, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    pwm_start(MOTOR_L_F, MOTOR_FREQ, BASE_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  }
  else
  {

    pwm_start(MOTOR_R_F, MOTOR_FREQ, BASE_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    pwm_start(MOTOR_L_F, MOTOR_FREQ, BASE_SPEED - gain, RESOLUTION_10B_COMPARE_FORMAT);
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

void pivot(int direction)
{
  int motor_start = millis();
  if (direction == LEFT)
  {
    pwm_start(MOTOR_R_F, MOTOR_FREQ, PIVOT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    pwm_start(MOTOR_L_B, MOTOR_FREQ, PIVOT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  }
  else if (direction == RIGHT)
  {
    pwm_start(MOTOR_L_F, MOTOR_FREQ, PIVOT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, PIVOT_SPEED, RESOLUTION_10B_COMPARE_FORMAT);
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
      pwm_start(MOTOR_R_F, MOTOR_FREQ, PIVOT_SPEED - 70, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

      pwm_start(MOTOR_L_B, MOTOR_FREQ, PIVOT_SPEED - 70, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    }
    if (millis() - motor_start > 200 && direction == RIGHT)
    {
      pwm_start(MOTOR_R_F, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_R_B, MOTOR_FREQ, PIVOT_SPEED - 50, RESOLUTION_10B_COMPARE_FORMAT);

      pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
      pwm_start(MOTOR_L_F, MOTOR_FREQ, PIVOT_SPEED - 50, RESOLUTION_10B_COMPARE_FORMAT);
    }
  }
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
  delay(100);
}

void closeGate()
{
  pwm_start(GATE_SERVO, SERVO_FREQ, GATE_CLOSED, RESOLUTION_10B_COMPARE_FORMAT);
  delay(200);
}

void depositCans()
{
  pwm_start(LEFT_SERVO, SERVO_FREQ, PLATFORM_UP_L, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(RIGHT_SERVO, SERVO_FREQ, PLATFORM_UP_R, RESOLUTION_10B_COMPARE_FORMAT);

  delay(900);

  pwm_start(LEFT_SERVO, SERVO_FREQ, PLATFORM_DOWN_L, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(RIGHT_SERVO, SERVO_FREQ, PLATFORM_DOWN_R, RESOLUTION_10B_COMPARE_FORMAT);

  delay(100);
}

void funMode()
{
  display.println("In funMode");
  display.display();
  openGate();
  delay(2000);
  closeGate();
  depositCans();
  while (true)
  {
  }
}
