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

const uint8_t rightStepPin = 2;
const uint8_t rightDirPin = 5;
const uint8_t leftStepPin = 4;
const uint8_t leftDirPin = 7;

GPT_Stepper leftStepper(leftStepPin, leftDirPin, 17000.0);
GPT_Stepper rightStepper(rightStepPin, rightDirPin, 17000.0);

float speed = 10000.0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\nStarting BalanceBot_Test.ino\n\n");

  leftStepper.init();
  rightStepper.init();

}

void loop() {

  static unsigned long pm = millis();
  unsigned long cm = millis();
  if(cm - pm >= 5000){
    pm=cm;
    leftStepper.setSpeed(speed);
    rightStepper.setSpeed(speed);
    speed = -speed;
  }

}
