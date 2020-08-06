#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible

// all pins are defined here

#define FUNSWITCH PB13
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
#define ENC_STRAIGHT_SPEED 920

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

#define KP 19
#define KD 10
#define THRESHOLD 580

#define BASE_SPEED 910

#define MOTOR_FREQ 2000
#define SERVO_FREQ 50
#define STOP 0

#define GATE_CLOSED 27.62
#define GATE_OPEN 110

#define PLATFORM_DOWN_R 126.76 
#define PLATFORM_UP_R 23.621 

#define PLATFORM_DOWN_L 23.621
#define PLATFORM_UP_L 126.76 

#define SONAR_LIMIT 55
#define SONAR_LIMIT_MID 30
#define SONAR_LIMIT_CLOSE 15

#define PIVOT_SPEED 920

#define LEFT 0
#define RIGHT 1

#define MAX_TURNS 7
#define FINAL_DUMP 59000