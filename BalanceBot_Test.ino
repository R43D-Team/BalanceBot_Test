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

#include "EEPROM.h"
#include "GPT_Stepper.h"

#include "ICM_20948.h"

#include "PID_DG.h"

#include "WiFiS3.h"


#include "R43D_Constants.h"
#include "IMU_Functions.h"
#include "AppConnection.h"
#include "EE_Storage.h"



/*
*  Pin Definitions
*/
#define NEWPINS
// Select your pin configuration
#if defined NEWPINS  // New shield board with hole

const uint8_t enablePin = 10;
const uint8_t rightStepPin = 8;
const uint8_t rightDirPin = 9;
const uint8_t leftStepPin = 11;
const uint8_t leftDirPin = 12;
#elif defined PROTOPINS  // Home Built Protoshield
const uint8_t enablePin = 10;
const uint8_t rightStepPin = 12;
const uint8_t rightDirPin = 11;
const uint8_t leftStepPin = 8;
const uint8_t leftDirPin = 9;
#else                    // Modified CNC shield
const uint8_t enablePin = 8;
const uint8_t rightStepPin = 4;
const uint8_t rightDirPin = 7;
const uint8_t leftStepPin = 2;
const uint8_t leftDirPin = 5;
#endif

/*
*  Stepper Motor Variables
*/
// GPT Stepper objects for the two motors
GPT_Stepper leftStepper(leftStepPin, leftDirPin, 34000, true);
GPT_Stepper rightStepper(rightStepPin, rightDirPin, 34000, false);

float maxSpeed = 10000.0;
float minSpeed = 10.0;
float speed = 0.0;
float steering = 0.0;


/*
*  PID Settings
*/

double anglePIDReportMultiplier = 1.0;
PID_Settings angleSettings = {
  .setpoint = 0,
  .Kp = 35.0,
  .Ki = 0.2,
  .Kd = 4.75,
  .outputMax = 5000,
  .outputMin = -5000,
  .direction = DIRECT
};


double speedPIDReportMultiplier = 10000.0;
PID_Settings speedSettings = {
  .setpoint = 0,
  .Kp = 0.02,
  .Ki = 0.0,
  .Kd = 0.001,
  .outputMax = 5,
  .outputMin = -5,
  .direction = DIRECT
};


// Used in Control Loop to enable PID from app and from loop in battery check
bool enable = false;
bool enabled = false;
bool enableSecondPID = false;
bool secondPIDEnabled = false;
bool standing = false;

// Instance variables for PID controllers
PID_Class anglePID(angleSettings);
PID_Class speedPID(speedSettings);

unsigned long controlLoopInterval = 10;

// Global variable to hold current pitch (shared between controlLoop and app code)
double pitch;


uint8_t heartLed = LED_BUILTIN;
unsigned long heartDelay = 1000;


uint32_t imuCalSaveTime = 0;
float batteryCalibrationFactor = 0.01349;

void stepperTest() {
  digitalWrite(enablePin, LOW);
  Serial.println("Left Forward");
  leftStepper.setSpeed(5000);
  delay(5000);
  leftStepper.setSpeed(-5000);
  Serial.println("Left Backward");
  delay(5000);
  leftStepper.setSpeed(0.0);
  rightStepper.setSpeed(5000);
  Serial.println("Right Forward");
  delay(5000);
  rightStepper.setSpeed(-5000);
  Serial.println("Right Backward");
  delay(5000);
  rightStepper.setSpeed(0.0);
  Serial.println("Test Complete");
  digitalWrite(enablePin, HIGH);
}

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
  // Load PID Settings
  if (getPIDSettings(EEPROM_ANGLE_SETTINGS, angleSettings)) {
    Serial.println("Good Angle PID Settings");
  } else {
    Serial.println("No Angle PID Settings Found");
  }
  if (getPIDSettings(EEPROM_SPEED_SETTINGS, speedSettings)) {
    Serial.println("Good Speed PID Settings");
  } else {
    Serial.println("No Speed PID Settings Found");
  }

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
  // Serial backdoor for testing
  if (Serial.available()) {
    Serial.println("Serial Backdoor");
    char c = Serial.read();
    if (c == '@') {
      Serial.println("Stepper Test");
      stepperTest();
    } else if (c == 'E') {
      enable = !enable;
      Serial.print("Setting enable to ");
      Serial.println(enable ? "True" : "False");
    } else if (c == 'B') {
      Serial.print("Raw Battery Analog ");
      Serial.println(analogRead(3));
      Serial.print("Calculated Reading ");
      Serial.println(readBattery());
    }
  }
  if (readBattery() < 10.0) {
    // Shut down motors and PID if battery is getting low
    if (enabled) {
      sendReturn(CC_MESSAGE, "Low Battery");
    }
    enable = false;
  }
  controlLoop();
  // Save IMU calibrations after two minutes if not calibrated
  if (!imuIsCalibrated() && (millis() - imuCalSaveTime > 120000)) {
    saveBiasStore();
    imuCalSaveTime = millis();
    sendReturn(CC_IMU_CAL, imuIsCalibrated());
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

double getCurrentSpeed() {
  // get an average of the speed of the two motors
  // subtract becasue one motor is backwards of the other
  return (leftStepper.getCurrentSpeed() + rightStepper.getCurrentSpeed()) / 2.0;
}

// This function is called from controlLoop to apply the PID output
void accelerate(double acc) {
  // I keep forgetting to fix the backwards output
  speed += acc;
  if (speed > maxSpeed) {
    speed = maxSpeed;
  }
  if (speed < -maxSpeed) {
    speed = -maxSpeed;
  }
  if (abs(speed) < minSpeed) {
    speed = 0;
  }
  leftStepper.setSpeed(speed + steering);
  rightStepper.setSpeed(speed - steering);
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
        sendReturn(CC_ENABLE, enabled);
      }
      // handle enable state change for second PID
      if (enableSecondPID != secondPIDEnabled) {
        secondPIDEnabled = enableSecondPID;
        if (secondPIDEnabled) {
          anglePID.bumplessStart(getCurrentSpeed(), 0.0, 21);
        } else {
          // reset angle setpoint when speed PID turns off.
          angleSettings.setpoint = 0.0;
          sendReturn(CC_ANGLEPID, PC_SETPOINT, angleSettings.setpoint);
        }
        sendReturn(CC_SECONDENABLE, secondPIDEnabled);
      }
      // if still enabled
      if (enabled) {
        if ((pitch > 45.0) || (pitch < -45.0)) {
          //  pitch is too much.  We fell over.
          standing = false;
          leftStepper.stop();
          rightStepper.stop();
          angleSettings.setpoint = 0.0;
          speedSettings.setpoint = 0.0;
          sendReturn(CC_ANGLEPID, PC_SETPOINT, angleSettings.setpoint);
        }
        if (standing) {
          // Only run the PID if standing.
          if (secondPIDEnabled) {
            accelerate(calculatePID());
          } else {
            // Get the output for the current pitch value
            double out = -anglePID.compute(pitch);
            // Call accelerate with the output.
            accelerate(out);
          }
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

double calculatePID() {
  //  use negative speed because motors are backwards
  double angle = speedPID.compute(getCurrentSpeed());
  angleSettings.setpoint = angle;
  double output = -anglePID.compute(pitch);
  return output;
}

float readBattery() {
  // The 12.915 was a manually calibrated value
  return analogRead(A3) * batteryCalibrationFactor;
}

/*
*
*   App Functions
*
*/

// Called when first making connection with app
void sendInitials() {
  sendReturn(CC_MESSAGE, "R43D Ready");
  sendReturn(CC_ANGLEPID, angleSettings, anglePIDReportMultiplier);
  sendReturn(CC_SPEEDPID, speedSettings, speedPIDReportMultiplier);
  sendReturn(CC_IMU_CAL, imuIsCalibrated());
  sendReturn(CC_ENABLE, enabled);
  sendReturn(CC_SECONDENABLE, secondPIDEnabled);
  sendReturn(CC_MAXSPEED, maxSpeed);
  sendReturn(CC_MINSPEED, minSpeed);
}


void serveReturns() {
  const uint32_t batteryInterval = 5000;
  const uint32_t tiltInterval = 200;

  uint32_t currentTime = millis();
  static uint32_t lastBatteryTime = millis();
  static uint32_t lastTiltTime = millis();

  if (currentTime - lastBatteryTime >= batteryInterval) {
    lastBatteryTime = currentTime;
    sendReturn(CC_VBATT, readBattery());
  } else if (currentTime - lastTiltTime >= tiltInterval) {
    lastTiltTime = currentTime;
    sendReturn(CC_TILT, pitch);
    sendReturn(CC_SPEED, getCurrentSpeed());
    static double oldSetpoint = angleSettings.setpoint;
    if (angleSettings.setpoint != oldSetpoint) {
      sendReturn(CC_ANGLEPID, PC_SETPOINT, angleSettings.setpoint);
      oldSetpoint = angleSettings.setpoint;
    }
    static double oldSpeedSetpoint = speedSettings.setpoint;
    if (speedSettings.setpoint != oldSpeedSetpoint) {
      sendReturn(CC_SPEEDPID, PC_SETPOINT, speedSettings.setpoint);
      oldSpeedSetpoint = speedSettings.setpoint;
    }
  }
}

// Called from handleClient when a command is received
// command will have the full command with both start and end markers intact.
void parseCommand(char *command) {
  // Serial.print("Parse Command :");
  // Serial.println(command);
  if (command[0] == '<') {
    switch (command[1]) {
      case CC_MAXSPEED:
        maxSpeed = atof(command + 3);
        break;
      case CC_MINSPEED:
        minSpeed = atof(command + 3);
        break;
      case CC_ANGLEPID:
      case CC_SPEEDPID:
        handlePIDReturn(command);
        break;
      case CC_DIRECTION:
        steering = atof(command + 3);
        sendReturn(CC_DIRECTION, steering);
        break;
      case CC_ENABLE:
        if (command[3] == '0') {
          enable = false;
        } else {
          enable = true;
        }
        break;
      case CC_SECONDENABLE:
        if (command[3] == '0') {
          enableSecondPID = false;
        } else {
          enableSecondPID = true;
        }
        break;
      case CC_IMU_CAL:
        if (command[3] == '0') {
          clearBiasStore();
          imuCalSaveTime = millis();  // so it will calibrate two minutes later.
        }
        sendReturn(CC_IMU_CAL, imuIsCalibrated());
        break;

      default:
        Serial.print("Unknown Message :");
        Serial.println(command);
    }
  }
}

void handlePIDReturn(char *buf) {
  PID_Settings *settings;
  char letter = buf[1];
  double multiplier = 1.0;
  if (letter == CC_ANGLEPID) {
    settings = &angleSettings;
    multiplier = anglePIDReportMultiplier;
  } else if (letter == CC_SPEEDPID) {
    settings = &speedSettings;
    multiplier = speedPIDReportMultiplier;
  } else {
    // bail out, it doesn't match
    return;
  }
  switch (buf[3]) {
    case PC_SETPOINT:
      settings->setpoint = atof(buf + 5);
      // This changes too fast to send back every time.
      //  It's handled in serveReturns
      // sendReturn(letter, PC_SETPOINT, settings->setpoint);
      break;
    case PC_KP:
      settings->Kp = atof(buf + 5) / multiplier;
      sendReturn(letter, PC_KP, settings->Kp, multiplier);
      break;
    case PC_KI:
      settings->Ki = atof(buf + 5) / multiplier;
      sendReturn(letter, PC_KI, settings->Ki, multiplier);
      break;
    case PC_KD:
      settings->Kd = atof(buf + 5) / multiplier;
      sendReturn(letter, PC_KD, settings->Kd, multiplier);
      break;
    case PC_OUTMAX:
      settings->outputMax = atof(buf + 5);
      sendReturn(letter, PC_OUTMAX, settings->outputMax);
      break;
    case PC_OUTMIN:
      settings->outputMin = atof(buf + 5);
      sendReturn(letter, PC_OUTMIN, settings->outputMin);
      break;
    case PC_EEPROM:
      unsigned int address;
      if (letter == CC_ANGLEPID) {
        address = EEPROM_ANGLE_SETTINGS;
      } else if (letter == CC_SPEEDPID) {
        address = EEPROM_SPEED_SETTINGS;
      } else {
        // Don't mess with EEPROM on mismatch
        break;
      }

      if (buf[5] == 'S') {
        storePIDSettings(address, *(settings));
      } else if (buf[5] == 'L') {
        getPIDSettings(address, *(settings));
        sendReturn(letter, *(settings), multiplier);
      }
      break;

    default:
      break;
  }
}
