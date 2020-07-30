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

Pid pid = Pid(KP);
SensorArray sensor_array = SensorArray();
Encoders encoders = Encoders(sensor_array);
Sonar_Logic sl = Sonar_Logic(&encoders);
bool returnToCircle = false;
bool returnToBin = false;
bool binToCircle = true;
Servo leftServo;
Servo rightServo;

bool first_iter; //for preventing from reaching stop condition on pivots before rotating off tape
int dropCanDelay = 0; // allow a few iterations of tape following before looking for cans again

void setup()
{
  pinSetup();
  attachInterrupt(digitalPinToInterrupt(PHOTOINTERRUPTER_R), handle_R_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(PHOTOINTERRUPTER_L), handle_L_interrupt, RISING);
  //displaySetup(display);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  Serial.begin(9600);
  // encoders.turnAroundContinuous();



  // sl.resetPWM();
  // delay(1000);


  pwm_start(RIGHT_SERVO, SERVO_FREQ, PLATFORM_DOWN_R, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(LEFT_SERVO, SERVO_FREQ, PLATFORM_DOWN_L, RESOLUTION_10B_COMPARE_FORMAT);

    delay(1000);
}

void loop()
{
  display.clearDisplay();
  display.setCursor(0, 0);

  while(true){
    pwm_start(RIGHT_SERVO, SERVO_FREQ, PLATFORM_DOWN_R, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(LEFT_SERVO, SERVO_FREQ, PLATFORM_DOWN_L, RESOLUTION_10B_COMPARE_FORMAT);

    delay(4000);

    pwm_start(RIGHT_SERVO, SERVO_FREQ, PLATFORM_UP_R, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(LEFT_SERVO, SERVO_FREQ, PLATFORM_UP_L, RESOLUTION_10B_COMPARE_FORMAT);

    delay(4000);
  }


  // for testing backup and pivot

  // while (true) {
  //   encoders.rightTurn90();
  //   delay(2000);
  // }
  // encoders.turnAroundContinuous();
  // encoders.driveStraightContinuous();

  // while (true) {
  //   encoders.driveStraight();
  //   delay(2000);
  // }
  // int g = map(analogRead(POT), 0, 1023, 50, 160);
  // pid.Kp = g;
  // display.println(pid.Kp);
  // display.display();

  //loop to see tcrt vals
  // while (true) {

  //   display.clearDisplay();
  //   display.setCursor(0, 0);
  //   int error = sensor_array.calculateError();

  //   int g = map(analogRead(POT), 0, 1023, 50, 160);
  // pid.Kp = g;
  // display.println(pid.Kp);
  //   display.println("Array");
  //   display.println(sensor_array.LFSensor[0]);
  //   display.println(sensor_array.LFSensor[1]);
  //   display.println(sensor_array.LFSensor[2]);
  //   display.println(sensor_array.LFSensor[3]);
  //   display.println(sensor_array.LFSensor[4]);
  //   display.println(sensor_array.digitalArr[5]);

  //   display.println(error);

  //   display.display();
  // }

  //going from bin to tape circle
  while (binToCircle)
  {

    display.setCursor(0, 0);
    display.clearDisplay();
    display.println("bin to circle");

    display.display();
    sensor_array.calculateError(); //this calcs error and updates all array vals in object

    motorStraight();
    // encoders.driveStraightContinuous();

    // if (sensor_array.digitalArr[0] == 1 || sensor_array.digitalArr[1] == 1 || sensor_array.digitalArr[2] == 1 || sensor_array.digitalArr[3] == 1 || sensor_array.digitalArr[4] == 1)
    if (sensor_array.anyFrontSensorOn())
    {
      binToCircle = false;
      delay(400);
      motorStop();
      delay(100);
      first_iter = true;
      encoders.rightTurn90();

      break;
    }
  }

  int error = sensor_array.calculateError();
  display.println("Array");
  display.println(sensor_array.LFSensor[0]);
  display.println(sensor_array.LFSensor[1]);
  display.println(sensor_array.LFSensor[2]);
  display.println(sensor_array.LFSensor[3]);
  display.println(sensor_array.LFSensor[4]);
  display.println(sensor_array.digitalArr[5]);

  display.println(error);

  display.display();

  pid.calculatePID(error);
  // display.println(pid.speed);
  // display.println(pid.slow_down);
  // display.display();
  motorPIDcontrol();

  if (returnToBin && sensor_array.digitalArr[5] == 1)
  {
    first_iter = true;
    motorStop();
    delay(400);
    // pivot(RIGHT);
    encoders.rightTurn90_bin();
    delay(2000);

    sl.driveTillFour();

    depositCans();

    //backing up and pivoting left until we meet tape
    encoders.backup();
    // encoders.turnAroundContinuous();
    first_iter = true;
    encoders.leftTurn90();

    dropCanDelay = 10;

    //temporary commented out
    returnToBin = false;

    //encoders.rightTurn90();
    // add function here for motorPID following and wait for collision sensor to perform lift
    //if no collision after a certain time just dump anyways?
  }

  if (dropCanDelay <= 0 && sl.pollSonar() < SONAR_LIMIT && !returnToBin)
  {
    sl.stopMotors();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("found shit");
    display.display();

    sl.pivotLeft();

    sl.goGetEm();

    delay(1000);

    // go back to the bin
    // for(int i  = 0; i < TURN_SCALE; i++)
    // {
    //   sl.pivotRight();
    // }
    // encoders.turnAround();
    encoders.rightTurn90();
    // encoders.driveStraightContinuous();

    returnToCircle = true; // this will make us return to the circle
    returnToBin = true;

    if (!sensor_array.anyFrontSensorOn())
    {

      while (returnToCircle)
      {
        display.setCursor(0, 0);
        display.clearDisplay();
        display.println("returning to circle");
        display.display();
        sensor_array.calculateError(); //this calcs error and updates all array vals in object

        // motorStraight();
        //using vals from sonar logic
        pwm_start(MOTOR_R_F, MOTOR_FREQ, STRAIGHT_R , RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

        pwm_start(MOTOR_L_F, MOTOR_FREQ, STRAIGHT_L, RESOLUTION_10B_COMPARE_FORMAT);
        pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

        if (sensor_array.digitalArr[0] == 1 || sensor_array.digitalArr[1] == 1 || sensor_array.digitalArr[2] == 1 || sensor_array.digitalArr[3] == 1 || sensor_array.digitalArr[4] == 1)
        {
          returnToCircle = false;
          delay(400);
          motorStop();
          delay(100);

          pivot(LEFT);

          break;
        }
      }
    }
  }

  dropCanDelay--;
}

void motorPIDcontrol()
{
  int gain = pid.speed;
  int slow_down = pid.slow_down;

  if (gain > 0)
  {
    pwm_start(MOTOR_R_F, MOTOR_FREQ, BASE_SPEED + gain, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_R_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    pwm_start(MOTOR_L_F, MOTOR_FREQ, BASE_SPEED - slow_down, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(MOTOR_L_B, MOTOR_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
  }
  else
  {
    pwm_start(MOTOR_R_F, MOTOR_FREQ, BASE_SPEED + slow_down, RESOLUTION_10B_COMPARE_FORMAT);
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
  while (true)
  {

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("pivoting");
    // readLFSsensors();
    sensor_array.calculateError();
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

    display.display();
    if (first_iter)
    {
      delay(1000);
    }

    if ((sensor_array.digitalArr[0] == 1 || sensor_array.digitalArr[1] == 1) && direction == LEFT)
    {
      motorStop();
      delay(200);
      break;
    }
    else if ((sensor_array.digitalArr[4] == 1 || sensor_array.digitalArr[3]) && direction == RIGHT)
    {
      motorStop();
      delay(200);
      break;
    }
    first_iter = false;
  }
}

void handle_R_interrupt()
{
  encoders.handle_R_interrupt();
  Serial.println("R");
}

void handle_L_interrupt()
{
  encoders.handle_L_interrupt();
  Serial.println("L");
}

void depositCans()
{
  pwm_start(LEFT_SERVO, SERVO_FREQ, PLATFORM_UP_L, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(RIGHT_SERVO, SERVO_FREQ, PLATFORM_UP_R, RESOLUTION_10B_COMPARE_FORMAT);

  delay(5000);

  sl.resetPWM();
}