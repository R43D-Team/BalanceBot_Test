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

// #include <TimerOne.h>

#define WIRE_PORT Wire1
#define AD0_VAL 1

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)

int status = WL_IDLE_STATUS;
WiFiServer server(2080);
IPAddress ipAddress(192, 168, 1, 81);

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

ICM_20948_I2C icm;
icm_20948_DMP_data_t data;

uint8_t heartLed = LED_BUILTIN;
unsigned long heartDelay = 1000;

unsigned long controlLoopInterval = 10;
bool controlLoopIntervalOverride = false;

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
  WIRE_PORT.setClock(400000);
  setupIMU();
  startWiFi();
  server.begin();

  leftStepper.init();
  rightStepper.init();
  leftStepper.setSpeed(0);
  rightStepper.setSpeed(0);

  Setpoint = 0.0;
  anglePID.SetOutputLimits(-34000, 34000);
  anglePID.SetSampleTime(20);
  Input = readPitch();
  anglePID.SetMode(AUTOMATIC);

  // Timer1.initialize(20000);
  // Timer1.attachInterrupt(controlLoop);

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
  // noInterrupts();
  // leftStepper.setAcceleration((acc >= 0.0) ? acc : -acc);
  // rightStepper.setAcceleration((acc >= 0.0) ? acc : -acc);
  // if (acc < 0) {
  //   leftStepper.setSpeed(-10000);
  //   rightStepper.setSpeed(-10000);
  // } else if (acc > 0) {
  //   leftStepper.setSpeed(10000);
  //   rightStepper.setSpeed(10000);
  // }
  // interrupts();
}


void controlLoop() {
  if (newData()) {
    Input = readPitch();
    if ((Input > 45.0) || (Input < -45.0)) {
      enable = false;
    }
    if (enable != enabled) {
      enabled = enable;
      if (enabled) {
        anglePID.SetMode(AUTOMATIC);
      } else {
        anglePID.SetMode(MANUAL);
        Output = 0;
        leftStepper.stop();
        rightStepper.stop();
        // leftStepper.setAcceleration(0);
        // rightStepper.setAcceleration(0);
      }
    }
    if (enabled) {
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
    }
  }
}

float readBattery() {
  return analogRead(A3) * (1.45 / 1023.0) * 12.915;
}

void setupIMU() {
  bool initialized = false;
  while (!initialized) {

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.

    icm.begin(WIRE_PORT, AD0_VAL);


    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(icm.statusString());

    if (icm.status != ICM_20948_Stat_Ok) {
      Serial.println(F("Trying again..."));

      delay(500);
    } else {
      initialized = true;
    }
  }
  Serial.println("\n\nConnected to IMU\n\n");
  bool success = true;
  success &= (icm.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (icm.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  // Enable the FIFO
  success &= (icm.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (icm.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (icm.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (icm.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success) {
    Serial.println("DMP enabled!");
  } else {
    Serial.println("Enable DMP failed!");
    Serial.println("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...");
    while (1)
      ;  // Do nothing more
  }
}


bool newData() {
  icm.readDMPdataFromFIFO(&data);
  controlLoopIntervalOverride = false;
  if ((icm.status == ICM_20948_Stat_Ok) || (icm.status == ICM_20948_Stat_FIFOMoreDataAvail))  // Was valid data available?
  {    
    if(icm.status == ICM_20948_Stat_FIFOMoreDataAvail){
      controlLoopIntervalOverride = true;
    }
    return true;
  }
  return false;
}

double readPitch() {

  double pitch = 0;

  if ((data.header & DMP_header_bitmap_Quat6) > 0)  // We have asked for GRV data so we should receive Quat6
  {
    // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
    // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
    // The quaternion data is scaled by 2^30.

    //Serial.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

    // Scale to +/- 1
    double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
    double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
    double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30

    // Convert the quaternions to Euler angles (roll, pitch, yaw)
    // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

    double q2sqr = q2 * q2;

    // // roll (x-axis rotation)
    // double t0 = +2.0 * (q0 * q1 + q2 * q3);
    // double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
    // double roll = atan2(t0, t1) * 180.0 / PI;

    // pitch (y-axis rotation)
    double t2 = +2.0 * (q0 * q2 - q3 * q1);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = asin(t2) * 180.0 / PI;

    // // yaw (z-axis rotation)
    // double t3 = +2.0 * (q0 * q3 + q1 * q2);
    // double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
    // double yaw = atan2(t3, t4) * 180.0 / PI;
  }

  return pitch;
}


void handleClient() {

  WiFiClient client = server.available();

  if (client) {
    char currentLine[64] = { 0 };
    uint8_t idx = 0;
    while (client.connected()) {
      delayMicroseconds(10);
      if (client.available()) {
        char c = client.read();
        // Serial.write(c);
        if (c == '\n') {

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (idx == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("<p>Use the forms below to set values</p>");
            client.print("<br>");
            client.print("<a href='/'>Reload</a>");
            client.print("<p>V-Batt: ");
            client.print(readBattery());
            client.print("</p>");
            client.print("<p>Current Pitch: ");
            client.print(Input);
            client.print("</p>");

            // Show a list of forms
            FormUpdatable::listForms(&client);

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {  // if you got a newline, then clear currentLine:
            // char buf[101];
            // currentLine.toCharArray(buf, 100);
            FormUpdatable::parse(currentLine);
            currentLine[0] = 0;
            idx = 0;
          }
        } else if (c != '\r') {
          if (idx < 63) {
            currentLine[idx] = c;
            currentLine[++idx] = 0;
          }
        }
      }
    }
    // close the connection:
    client.stop();
  }
}


void startWiFi() {
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  if (WiFi.firmwareVersion() < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  WiFi.config(ipAddress);

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);  // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}
