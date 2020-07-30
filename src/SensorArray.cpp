#include <Arduino.h>
#include "SensorArray.h"
#include <../src/consts/const.h>

SensorArray::SensorArray() {
}

int SensorArray::calculateError() {
    LFSensor[0] = analogRead(TCRT1);
    LFSensor[1] = analogRead(TCRT2);
    LFSensor[2] = analogRead(TCRT3);
    LFSensor[3] = analogRead(TCRT4);
    LFSensor[4] = analogRead(TCRT5);
    if (digitalRead(TCRT_DIGITAL) == 1) {
        digitalArr[5] = 0;
    } else {
        digitalArr[5] = 1;
    }

    for (int i = 0; i <= 4; i++)
    {
        if (LFSensor[i] > THRESHOLD)
        {
        digitalArr[i] = 1;
        }
        else
        {
        digitalArr[i] = 0;
        }
    }

    if ((digitalArr[0] == 0) && (digitalArr[1] == 0) && (digitalArr[2] == 0) && (digitalArr[3] == 0) && (digitalArr[4] == 1))
        error = -4.5;
        // error = -4;

    else if ((digitalArr[0] == 0) && (digitalArr[1] == 0) && (digitalArr[2] == 0) && (digitalArr[3] == 1) && (digitalArr[4] == 1))
        error = -4;

    else if ((digitalArr[0] == 0) && (digitalArr[1] == 0) && (digitalArr[2] == 0) && (digitalArr[3] == 1) && (digitalArr[4] == 0))
        error = -3;

    else if ((digitalArr[0] == 0) && (digitalArr[1] == 0) && (digitalArr[2] == 1) && (digitalArr[3] == 1) && (digitalArr[4] == 0))
        error = -2;

    else if ((digitalArr[0] == 0) && (digitalArr[1] == 0) && (digitalArr[2] == 1) && (digitalArr[3] == 0) && (digitalArr[4] == 0))
        error = 0;

    else if ((digitalArr[0] == 0) && (digitalArr[1] == 1) && (digitalArr[2] == 1) && (digitalArr[3] == 0) && (digitalArr[4] == 0))
        error = 2;

    else if ((digitalArr[0] == 0) && (digitalArr[1] == 1) && (digitalArr[2] == 0) && (digitalArr[3] == 0) && (digitalArr[4] == 0))
        error = 3;

    else if ((digitalArr[0] == 1) && (digitalArr[1] == 1) && (digitalArr[2] == 0) && (digitalArr[3] == 0) && (digitalArr[4] == 0))
        error = 4;

    else if ((digitalArr[0] == 1) && (digitalArr[1] == 0) && (digitalArr[2] == 0) && (digitalArr[3] == 0) && (digitalArr[4] == 0))
        error = 4.5;
        // error = 4;

    return error;
}

bool SensorArray::anyFrontSensorOn() {
    this->calculateError();
    return (digitalArr[0] == 1 || digitalArr[1] == 1 || digitalArr[2] == 1 || digitalArr[3] == 1 || digitalArr[4] == 1);
}

// void SensorArray::calculateThresholds() {
    
// }