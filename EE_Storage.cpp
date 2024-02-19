#include "EE_Storage.h"


unsigned char PID_Settings_Store::calculateSum() {
  byte rv = header;
  for (int i = 0; i < sizeof(PID_Settings); i++) {
    rv += settings[i];
  }
  return rv;
}

void PID_Settings_Store::updateSum() {
  sum = calculateSum();
}

bool PID_Settings_Store::validateSum() {
  return calculateSum() == sum;
}

void storePIDSettings(unsigned int address, PID_Settings &settings) {
  PID_Settings_Store store;
  memcpy(&(store.settings), &settings, sizeof(PID_Settings));
  store.updateSum();
  EEPROM.put(address, store);
}

bool getPIDSettings(unsigned int address, PID_Settings &settings) {
  bool rv = false;
  PID_Settings_Store store;
  EEPROM.get(address, store);
  if (store.validateSum()) {
    memcpy(&settings, &(store.settings), sizeof(PID_Settings));
    rv = true;
  }
  return rv;
}