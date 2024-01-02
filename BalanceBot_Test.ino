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

#include <PID_v1.h>

#include "WiFiS3.h"

#include "arduino_secrets.h"
#include "FormUpdatable.h"

#include "IMU_Functions.h"
#include "R43D_WebPage.h"


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

float maxSpeed = 10000.0;
float speed;

FormUpdatableValue fuMs(maxSpeed, "maxSpeed");
FormUpdatableValue fuSp(Setpoint, "Setpoint");
FormUpdatableValue fuKp(Kp, "Kp");
FormUpdatableValue fuKi(Ki, "Ki");
FormUpdatableValue fuKd(Kd, "Kd");
FormUpdatableValue fuEn(enable, "enable");

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
  anglePID.SetOutputLimits(-34000, 34000);
  anglePID.SetSampleTime(20);
  Input = readPitch();
  anglePID.SetMode(AUTOMATIC);

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
  static uint32_t pm = millis();
  uint32_t cm = millis();
  if (controlLoopIntervalOverride || (cm - pm >= controlLoopInterval)) {
    pm = cm;
    controlLoop();
  }
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
  leftStepper.setSpeed(speed);
  rightStepper.setSpeed(speed);
}


void controlLoop() {
  if (newData()) {
    Input = readPitch();
    if (enable != enabled) {
      enabled = enable;
      if (enabled) {
        anglePID.SetMode(AUTOMATIC);
      } else {
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
        static double oldSetpoint = 0;
        // Reinitialize if setpoint changes
        if (Setpoint != oldSetpoint) {
          oldSetpoint = Setpoint;
          anglePID.SetMode(MANUAL);
          anglePID.SetMode(AUTOMATIC);
        }
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

float readBattery() {
  return analogRead(A3) * (1.45 / 1023.0) * 12.915;
}
