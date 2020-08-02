#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible

// all pins are defined here

#define ECHO_PIN PB14
#define TRIG_PIN PB15

// #define MOTOR_R_F PB_9
// #define MOTOR_R_B PB_8
// #define MOTOR_L_F PA_2
// #define MOTOR_L_B PA_3

#define PHOTOINTERRUPTER_R PA15
#define PHOTOINTERRUPTER_L PA12
#define ENC_R_FWD 61
#define ENC_L_FWD 61
#define ENC_STRAIGHT_SPEED 950

#define MOTOR_L_F PB_9
#define MOTOR_L_B PB_8
#define MOTOR_R_F PA_2
#define MOTOR_R_B PA_3

#define LEFT_SERVO PB_1
#define RIGHT_SERVO PB_0
#define GATE_SERVO PA_8

#define POT PA7

#define TCRT1 PA6
#define TCRT2 PA5
#define TCRT3 PA4
#define TCRT4 PA1
#define TCRT5 PA0

#define TCRT_DIGITAL PB10

#define FUNSWITCH PB13

#define KP 16
#define KD 10
//#define THRESHOLD 725
#define THRESHOLD 730
#define BASE_SPEED 910

#define MOTOR_FREQ 2000
#define SERVO_FREQ 50
#define STOP 0

#define STRAIGHT_L 780
#define STRAIGHT_R 820

#define LEFT_TURN_L 800
#define LEFT_TURN_R 0

#define TURN_TIME 1390
#define TURN_SCALE 10.0

#define STRAIGHT_LENGTH 10

#define GATE_CLOSED 27.62
#define GATE_OPEN 90 //85.76
#define SERVO_STEP 10

#define PLATFORM_DOWN_R 126.76

#define PLATFORM_UP_R 23.621

#define PLATFORM_DOWN_L 23.621

#define PLATFORM_UP_L 126.76

// #define PLATFORM_DOWN_R 180
// #define PLATFORM_UP_R 90

// #define PLATFORM_DOWN_L 0
// #define PLATFORM_UP_L 90

#define SONAR_LIMIT 55
#define SONAR_LIMIT_CLOSE 15
#define SONAR_LIMIT_MID 30
#define SONAR_SAFETY_OFFSET 7
#define CAN_LIMIT 32.0
#define CAN_TOO_CLOSE 25.0

#define PIVOT_SPEED 920

#define LEFT 0
#define RIGHT 1

#define MAX_TURNS 7
