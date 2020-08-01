#include <../src/sonar_logic/sonar_logic.h>

Sonar_Logic::Sonar_Logic(Encoders * encoder){
    this->count = 0;
    this->backedUp = 0;
    this->pickedUp = false;
    this->encoder = encoder;

    resetPWM();
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

void Sonar_Logic::resetPWM()
{
  // setting gate closed, platform down and motors initially stopped
  pwm_start(GATE_SERVO, SERVO_FREQ, GATE_CLOSED, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(PB_1, SERVO_FREQ, PLATFORM_DOWN_L, RESOLUTION_10B_COMPARE_FORMAT);
  pwm_start(PB_0, SERVO_FREQ, PLATFORM_DOWN_R, RESOLUTION_10B_COMPARE_FORMAT);
  stopMotors();

  delay(500);
}