/*

BalanceBot_Test.ino  --  Test code for robot I'm building with @PickyBiker (forum.arduino.cc)
     Copyright (C) 2023  David C.

     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.

     */


#include "GPT_Stepper.h"

#include "ICM_20948.h"

#include <PID_vZ.h>

#include "WiFiS3.h"

#include "arduino_secrets.h"
#include "FormUpdatable.h"

#include "IMU_Functions.h"
#include "R43D_WebPage.h"

const uint8_t enablePin = 8;
const uint8_t rightStepPin = 2;
const uint8_t rightDirPin = 5;
const uint8_t leftStepPin = 4;
const uint8_t leftDirPin = 7;

GPT_Stepper leftStepper(leftStepPin, leftDirPin, 34000, true);
GPT_Stepper rightStepper(rightStepPin, rightDirPin, 34000, false);

int enable = 0;
int enabled = 0;

double Setpoint, Input, Output;
double Kp = 50.0;
double Ki = 0;
double Kd = 3.0;
double pidOutputLimit = 5000;

float maxSpeed = 10000.0;
float minSpeed = 10.0;
float speed;

int pidSampleTimeMs = 20;

PID anglePID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

uint8_t heartLed = LED_BUILTIN;
unsigned long heartDelay = 1000;

unsigned long controlLoopInterval = 10;
bool controlLoopIntervalOverride = false;

bool imuCalibrated = false;
uint32_t imuCalSaveTime = 0;

bool standing;

void setup() {
  //  Three flashes to start program:
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);
  pinMode(heartLed, OUTPUT);
  for (int i = 0; i < 3; i++) {
    delay(100);
    digitalWrite(heartLed, HIGH);
    delay(100);
    digitalWrite(heartLed, LOW);
  }
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\nStarting BalanceBot_Test.ino\n\n");
  WIRE_PORT.begin();
  WIRE_PORT.setClock(1000000);
  setupIMU();
  imuCalibrated = loadBiasStore();
  startWiFi();

  leftStepper.init();
  rightStepper.init();
  leftStepper.setSpeed(0);
  rightStepper.setSpeed(0);

  Setpoint = 0.0;
  anglePID.SetOutputLimits(-pidOutputLimit, pidOutputLimit);
  anglePID.SetSampleTime(pidSampleTimeMs);
  Input = readPitch();
  anglePID.SetMode(MANUAL);

  analogReference(AR_INTERNAL);
  // Three more flashes at end of setup.
  for (int i = 0; i < 3; i++) {
    delay(100);
    digitalWrite(heartLed, HIGH);
    delay(100);
    digitalWrite(heartLed, LOW);
  }
}

void loop() {
  handleClient();
  heartbeat();
  if (readBattery() < 10.0) {
    enable = false;
  }
  controlLoop();
  if (!imuCalibrated && (millis() - imuCalSaveTime > 120000)) {
    imuCalibrated = saveBiasStore();
    imuCalSaveTime = millis();
  }
}

void heartbeat() {
  static unsigned long pm = millis();
  unsigned long cm = millis();
  if (cm - pm >= heartDelay) {
    pm = cm;
    static uint8_t state = LOW;
    state = 1 - state;
    digitalWrite(heartLed, state);
  }
}

void accelerate(double acc) {
  speed -= acc;
  if (speed > maxSpeed) {
    speed = maxSpeed;
  }
  if (speed < -maxSpeed) {
    speed = -maxSpeed;
  }
  if (abs(speed) < minSpeed) {
    speed = 0;
  }
  leftStepper.setSpeed(speed);
  rightStepper.setSpeed(speed);
}


void controlLoop() {

  static uint32_t pm = millis();
  uint32_t cm = millis();
  while (controlLoopIntervalOverride || (cm - pm >= controlLoopInterval)) {
    pm = cm;
    if (newData()) {
      Input = readPitch();
      if (enable != enabled) {
        enabled = enable;
        if (enabled) {
          anglePID.SetMode(AUTOMATIC);
          digitalWrite(enablePin, LOW);
        } else {
          digitalWrite(enablePin, HIGH);
          anglePID.SetMode(MANUAL);
          Output = 0;
          leftStepper.stop();
          rightStepper.stop();
        }
      }
      if (enabled) {
        if ((Input > 45.0) || (Input < -45.0)) {
          standing = false;
          leftStepper.stop();
          rightStepper.stop();
        }
        if (standing) {
          anglePID.SetTunings(Kp, Ki, Kd);
          // static double oldSetpoint = 0;
          // Reinitialize if setpoint changes
          // if (Setpoint != oldSetpoint) {
          //   oldSetpoint = Setpoint;
          //   anglePID.SetMode(MANUAL);
          //   anglePID.SetMode(AUTOMATIC);
          // }
          anglePID.SetOutputLimits(-pidOutputLimit, pidOutputLimit);
          anglePID.Compute();
          accelerate(Output);
        } else {
          if ((Input > -5.0) && (Input < 5.0)) {
            standing = true;
          }
        }
      }
    }
  }
}

float readBattery() {
  return analogRead(A3) * (1.45 / 1023.0) * 12.915;
}

void sendReturn(WiFiClient* client, char command, double value) {
  char buf[16];
  char num[10];
  dtostrf(value, 2, 2, num);
  snprintf(buf, 16, "<%c,%s>", command, num);
  client->println(buf);
}

void sendReturn(WiFiClient* client, char command, boolean value) {
  char buf[16];
  snprintf(buf, 16, "<%c,%s>", command, value? "True" : "False");
  client->println(buf);
}

void sendInitials(WiFiClient* client) {
  sendReturn(client, 'P', Kp);
  sendReturn(client, 'D', Kd);
  sendReturn(client, 'I', Ki);
  sendReturn(client, 'M', maxSpeed);
  sendReturn(client, 'm', minSpeed);
  sendReturn(client, 'L', pidOutputLimit);
  sendReturn(client, 'S', Setpoint);
  sendReturn(client, 'c', imuCalibrated);
  
}

void serveReturns(WiFiClient* client) {
  const uint32_t batteryInterval = 5000;
  const uint32_t tiltInterval = 200;

  uint32_t currentTime = millis();
  static uint32_t lastBatteryTime = millis();
  static uint32_t lastTiltTime = millis();

  static boolean firstTime = true;
  if (firstTime) {
    firstTime = false;
    sendInitials(client);
  }

  if (currentTime - lastBatteryTime >= batteryInterval) {
    lastBatteryTime = currentTime;
    sendReturn(client, 'B', readBattery());
    // dtostrf(readBattery(), 2, 2, num);
    // snprintf(buf, 16, "<B,%s>", num);
    // client->println(buf);
    // Serial.print("tr: ");
    // Serial.println(millis() - currentTime);
    // client->print("<B,");
    // client->print(readBattery());
    // client->println(">");
  } else if (currentTime - lastTiltTime >= tiltInterval) {
    lastTiltTime = currentTime;
    sendReturn(client, 'T', Input);
    //   client->print("<T,");
    //   client->print(Input);
    //   client->println(">");
  }
}

void parseCommand(char* command) {
  // Serial.print("Parse Command :");
  // Serial.println(command);
  if (command[0] == '<') {
    switch (command[1]) {
      case 'S':
        Setpoint = atof(command + 3);
        break;
      case 'P':
        Kp = atof(command + 3);
        break;
      case 'I':
        Ki = atof(command + 3);
        break;
      case 'D':
        Kd = atof(command + 3);
        break;
      case 'M':
        maxSpeed = atof(command + 3);
        break;
      case 'm':
        minSpeed = atof(command + 3);
        break;
      case 'L':
        pidOutputLimit = atof(command + 3);
        break;
      case 'E':
        if (command[3] == '0') {
          enable = false;
        } else {
          enable = true;
        }
        break;
      default:
        Serial.print("Unknown Message :");
        Serial.println(command);
    }
  }
}
