#ifndef EE_STORAGE_H
#define EE_STORAGE_H

#include "Arduino.h"
#include "PID_DG.h"
#include "EEPROM.h"

struct PID_Settings_Store {
  byte header = 0x42;
  unsigned char settings[sizeof(PID_Settings)];
  byte sum = 0;

  void updateSum();
  bool validateSum();
  unsigned char calculateSum();
};

bool getPIDSettings(unsigned int address, PID_Settings &settings);
void storePIDSettings(unsigned int address, PID_Settings &settings);

#endif // EE_STORAGE_H