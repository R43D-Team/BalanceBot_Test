#ifndef R43D_WEBPAGE_H
#define R43D_WEBPAGE_H


#include "Arduino.h"
#include "WiFiS3.h"

void handleClient();

void startWiFi();

void printWiFiStatus();

float readBattery();

#endif //R43D_WEBPAGE_H