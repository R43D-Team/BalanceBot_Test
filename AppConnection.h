#ifndef R43D_WEBPAGE_H
#define R43D_WEBPAGE_H


#include "Arduino.h"
#include "WiFiS3.h"

void sendReturn(char command, char* value);
void sendReturn(char command, char param, double value);
void sendReturn(char command, double value);
void sendReturn(char command, boolean value);

void handleClient();

void startWiFi();

void printWiFiStatus();

float readBattery();

#endif //R43D_WEBPAGE_H