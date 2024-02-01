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

/*
*  Pin Definitions
*/
#define NEWPINS
// Select the pins for the robot configuration
#ifdef NEWPINS
const uint8_t enablePin = 10;
const uint8_t rightStepPin = 8;
const uint8_t rightDirPin = 9;
const uint8_t leftStepPin = 12;
const uint8_t leftDirPin = 11;
#else
const uint8_t enablePin = 8;
const uint8_t rightStepPin = 2;
const uint8_t rightDirPin = 5;
const uint8_t leftStepPin = 4;
const uint8_t leftDirPin = 7;
#endif

/*
*  Stepper Motor Variables
*/
// GPT Stepper objects for the two motors
GPT_Stepper leftStepper(leftStepPin, leftDirPin, 34000, true);
GPT_Stepper rightStepper(rightStepPin, rightDirPin, 34000, false);

float maxSpeed = 10000.0;
float minSpeed = 10.0;
float speed;


/*
*  PID Settings
*/
PID_Settings angleSettings = {
  .setpoint = 0,
  .Kp = 72.0,
  .Ki = 0.8,
  .Kd = 10.0,
  .outputMax = 5000,
  .outputMin = -5000,
  .direction = DIRECT
};

// Used in Control Loop to enable PID from app and from loop in battery check
int enable = 0;
int enabled = 0;

bool standing;

// Instance variables for PID controllers
PID_Class anglePID(angleSettings);

unsigned long controlLoopInterval = 10;

// Global variable to hold current pitch (shared between controlLoop and app code)
double pitch;


uint8_t heartLed = LED_BUILTIN;
unsigned long heartDelay = 1000;


uint32_t imuCalSaveTime = 0;


void setup() {
  //  Three flashes to start program:
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);
  pinMode(heartLed, OUTPUT);
  // Three flashes to start setup...
  for (int i = 0; i < 3; i++) {
    delay(100);
    digitalWrite(heartLed, HIGH);
    delay(100);
    digitalWrite(heartLed, LOW);
  }
  //  Start communication peripherals
  // Serial
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\nStarting BalanceBot_Test.ino\n\n");
  // I2C
  WIRE_PORT.begin();
  WIRE_PORT.setClock(1000000);
  // start the IMU before starting WiFi since WiFi takes so long
  setupIMU();
  startWiFi();
  // initialize the motors
  leftStepper.init();
  rightStepper.init();
  leftStepper.setSpeed(0);
  rightStepper.setSpeed(0);
  // take an initial pitch reading
  pitch = readPitch();
  // Use 1.45V internal reference for stable battery voltage readings
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
    // Shut down motors and PID if battery is getting low
    enable = false;
  }
  controlLoop();
  // Save IMU calibrations after two minutes if not calibrated
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

// This function is called from controlLoop to apply the PID output
void accelerate(double acc) {
  // I keep forgetting to fix the backwards output
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
  if (cm - pm >= controlLoopInterval) {
    // if the IMU has new data. 
    if (newData()) {
      // reset controlLoopInterval if there's new data
      // This means that the IMU read rate is really setting our PID rate
      pm = cm;  
      // get the pitch
      pitch = readPitch();
      // handle enable state change
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
      // if still enabled
      if (enabled) {
        if ((pitch > 45.0) || (pitch < -45.0)) {
          //  pitch is too much.  We fell over. 
          standing = false;
          leftStepper.stop();
          rightStepper.stop();
        }
        if (standing) {
          // Only run the PID if standing.  
          // Enforce a time for now in case of ControlLoopIntervalOverride

          // Get the output for the current pitch value
          double out = anglePID.compute(pitch);
          // Call accelerate with the output. 
          accelerate(out);
        } else {
          // standing is false, so check to see if we've been righted. 
          if ((pitch > -5.0) && (pitch < 5.0)) {
            standing = true;
          }
        }
      }
    }
  }
}

float readBattery() {
  // The 12.915 was a manually calibrated value
  return analogRead(A3) * (1.45 / 1023.0) * 12.915;
}

/*
*
*   App Functions
*
*/

// Called when first making connection with app
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

// Called from handleClient.  Don't serve too much.  Try to combine into one send.  
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

// Called from handleClient when a command is received
// command will have the full command with both start and end markers intact. 
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
