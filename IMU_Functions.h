#ifndef IMU_FUNCTIONS_H
#define IMU_FUNCTIONS_H

#include "Arduino.h"
#include "ICM_20948.h"

#include "EEPROM.h"

#define WIRE_PORT Wire1
#define AD0_VAL 1
#define EEPROM_BIAS_STORE 32

struct biasStore
{
  int32_t header = 0x42;
  int32_t biasGyroX = 0;
  int32_t biasGyroY = 0;
  int32_t biasGyroZ = 0;
  int32_t biasAccelX = 0;
  int32_t biasAccelY = 0;
  int32_t biasAccelZ = 0;
  int32_t biasCPassX = 0;
  int32_t biasCPassY = 0;
  int32_t biasCPassZ = 0;
  int32_t sum = 0;
};

void setupIMU();

bool newData();

double readPitch();

void updateBiasStoreSum(biasStore *store);
bool isBiasStoreValid(biasStore *store);
bool saveBiasStore();
bool loadBiasStore();

#endif //IMU_FUNCTIONS_H