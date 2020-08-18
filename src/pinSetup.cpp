#include "pinSetup.h"

void pinSetup()
{
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);

    pinMode(MOTOR_R_F, OUTPUT);
    pinMode(MOTOR_R_B, OUTPUT);
    pinMode(MOTOR_L_F, OUTPUT);
    pinMode(MOTOR_L_B, OUTPUT);

    pinMode(LEFT_SERVO, OUTPUT);
    pinMode(RIGHT_SERVO, OUTPUT);
    pinMode(GATE_SERVO, OUTPUT);

    pinMode(POT, INPUT_ANALOG);

    pinMode(TCRT1, INPUT_ANALOG);
    pinMode(TCRT2, INPUT_ANALOG);
    pinMode(TCRT3, INPUT_ANALOG);
    pinMode(TCRT4, INPUT_ANALOG);
    pinMode(TCRT5, INPUT_ANALOG);

    pinMode(TCRT_DIGITAL, INPUT);

    pinMode(PHOTOINTERRUPTER_R, INPUT_PULLUP);
    pinMode(PHOTOINTERRUPTER_L, INPUT_PULLUP);

    pinMode(FUNSWITCH,INPUT_PULLUP);
}