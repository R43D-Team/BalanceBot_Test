#ifndef IMU_FUNCTIONS_H
#define IMU_FUNCTIONS_H

#include "Arduino.h"
#include "ICM_20948.h"

#define WIRE_PORT Wire1
#define AD0_VAL 1

void setupIMU();

bool newData();

double readPitch();


#endif //IMU_FUNCTIONS_H