#ifndef R43D_CONSTANTS_H
#define R43D_CONSTANTS_H

#include "Arduino.h"
#include "EE_Storage.h"

#define EEPROM_ANGLE_SETTINGS (128)
#define EEPROM_SPEED_SETTINGS (128 + sizeof(PID_Settings_Store))

enum ControlCode_t : char {
  CC_MAXSPEED = 'M',
  CC_MINSPEED = 'm',
  CC_ENABLE = 'E',
  CC_SECONDENABLE = 'e',
  CC_DIRECTION = 'D',
  CC_IMU_CAL = 'c',
  CC_VBATT = 'B',
  CC_TILT = 'T',
  CC_SPEED = 's',
  CC_ANGLEPID = 'A',
  CC_SPEEDPID = 'S',
  CC_MESSAGE = '?'
};

enum PIDCode_t : char {
  PC_SETPOINT = 'S',
  PC_KP = 'P',
  PC_KI = 'I',
  PC_KD = 'D',
  PC_OUTMAX = 'M',
  PC_OUTMIN = 'm',
  PC_EEPROM = 'e'
};

#endif // R43D_CONSTANTS_H