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

#include "PID_DG.h"

#include "WiFiS3.h"

#include "IMU_Functions.h"
#include "AppConnection.h"

const uint8_t enablePin = 10;
const uint8_t rightStepPin = 8;
const uint8_t rightDirPin = 9;
const uint8_t leftStepPin = 12;
const uint8_t leftDirPin = 11;

GPT_Stepper leftStepper(leftStepPin, leftDirPin, 34000, true);
GPT_Stepper rightStepper(rightStepPin, rightDirPin, 34000, false);

int enable = 0;
int enabled = 0;

PID_Settings angleSettings = {
  .setpoint = 0,
  .Kp = 72.0,
  .Ki = 0.8,
  .Kd = 10.0,
  .outputMax = 5000,
  .outputMin = -5000,
  .direction = DIRECT
};

double pitch;

float maxSpeed = 10000.0;
float minSpeed = 10.0;
float speed;

int pidSampleTimeMs = 20;

PID_Class anglePID(angleSettings);

uint8_t heartLed = LED_BUILTIN;
unsigned long heartDelay = 1000;

unsigned long controlLoopInterval = 10;
bool controlLoopIntervalOverride = false;


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
  startWiFi();

  leftStepper.init();
  rightStepper.init();
  leftStepper.setSpeed(0);
  rightStepper.setSpeed(0);

  pitch = readPitch();

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
  if (!imuIsCalibrated() && (millis() - imuCalSaveTime > 120000)) {
    saveBiasStore();
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

    if (newData()) {
      pitch = readPitch();
      if (enable != enabled) {
        enabled = enable;
        if (enabled) {
          anglePID.bumplessStart(pitch, 0.0, 21);
          digitalWrite(enablePin, LOW);
        } else {
          digitalWrite(enablePin, HIGH);
          leftStepper.stop();
          rightStepper.stop();
        }
      }
      if (enabled) {
        if ((pitch > 45.0) || (pitch < -45.0)) {
          standing = false;
          leftStepper.stop();
          rightStepper.stop();
        }
        if (standing && ((cm - pm) > 10)) {
          double out = anglePID.compute(pitch);
          accelerate(out);
        } else {
          if ((pitch > -5.0) && (pitch < 5.0)) {
            standing = true;
          }
        }
      }
    }
    pm = cm;
  }
}

float readBattery() {
  return analogRead(A3) * (1.45 / 1023.0) * 12.915;
}



void sendInitials() {
  sendReturn('P', angleSettings.Kp);
  sendReturn('D', angleSettings.Kd);
  sendReturn('I', angleSettings.Ki);
  sendReturn('M', maxSpeed);
  sendReturn('m', minSpeed);
  sendReturn('L', angleSettings.outputMax);
  sendReturn('S', angleSettings.setpoint);
  sendReturn('c', imuIsCalibrated());
}

void serveReturns() {
  const uint32_t batteryInterval = 5000;
  const uint32_t tiltInterval = 200;

  uint32_t currentTime = millis();
  static uint32_t lastBatteryTime = millis();
  static uint32_t lastTiltTime = millis();

  if (currentTime - lastBatteryTime >= batteryInterval) {
    lastBatteryTime = currentTime;
    sendReturn('B', readBattery());
  } else if (currentTime - lastTiltTime >= tiltInterval) {
    lastTiltTime = currentTime;
    sendReturn('T', pitch);
  }
}

void parseCommand(char* command) {
  // Serial.print("Parse Command :");
  // Serial.println(command);
  if (command[0] == '<') {
    switch (command[1]) {
      case 'S':
        angleSettings.setpoint = atof(command + 3);
        break;
      case 'P':
        angleSettings.Kp = atof(command + 3);
        break;
      case 'I':
        angleSettings.Ki = atof(command + 3);
        break;
      case 'D':
        angleSettings.Kd = atof(command + 3);
        break;
      case 'M':
        maxSpeed = atof(command + 3);
        break;
      case 'm':
        minSpeed = atof(command + 3);
        break;
      case 'L':
        {
          double set = atof(command + 3);
          angleSettings.outputMax = set;
          angleSettings.outputMin = set;
          break;
        }
      case 'E':
        if (command[3] == '0') {
          enable = false;
        } else {
          enable = true;
        }
        break;
      case 'c':
        if (command[3] == '0') {
          clearBiasStore();
          imuCalSaveTime = millis();  // so it will calibrate two minutes later.
        }
      default:
        Serial.print("Unknown Message :");
        Serial.println(command);
    }
  }
}
