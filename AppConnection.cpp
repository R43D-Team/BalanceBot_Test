#include "AppConnection.h"

#define RETURN_BUFFER_SIZE 128

char ssid[] = "R43D_Remote_AP";
char pass[] = "";

int status = WL_IDLE_STATUS;
WiFiServer server(2080);
IPAddress ipAddress(192, 168, 4, 34);

WiFiClient client;

extern void parseCommand(char* command);
extern void serveReturns();
extern void sendInitials();

char returnBuffer[RETURN_BUFFER_SIZE];

void sendReturn(char command, char* value) {
  char buf[32];
  snprintf(buf, 32, "<%c,%s>", command, value);

  int len = strlen(buf);
  if (strlen(returnBuffer) + strlen(buf) <= RETURN_BUFFER_SIZE) {
    strcat(returnBuffer, buf);
  } else {
    // allow blocking for a full buffer
    // so don't send long strings of stuff
    // while robot is balancing
    sendReturnBuffer();
    strcat(returnBuffer, buf);
  }
}

void sendReturn(char command, char param, double value) {
  char num[24];
  num[0] = param;
  num[1] = ',';
  dtostrf(value, 2, 4, num + 2);
  sendReturn(command, num);
}

void sendReturn(char command, double value) {
  char num[16];
  dtostrf(value, 2, 2, num);
  sendReturn(command, num);
}

void sendReturn(char command, boolean value) {
  char buf[7];
  snprintf(buf, 7, "%s", value ? "True" : "False");
  sendReturn(command, buf);
}

void sendReturn(char command, PID_Settings& settings) {

  sendReturn(command, 'P', settings.Kp);
  sendReturn(command, 'D', settings.Kd);
  sendReturn(command, 'I', settings.Ki);
  sendReturn(command, 'M', settings.outputMax);
  sendReturn(command, 'm', settings.outputMin);
  sendReturn(command, 'S', settings.setpoint);
}

void sendReturnBuffer() {
  if (strlen(returnBuffer) > 0) {
    client.println(returnBuffer);
    returnBuffer[0] = 0;
  }
}

void handleClient() {

  // if (status != WiFi.status()) {
  //   // it has changed update the variable
  //   status = WiFi.status();

  //   if (status == WL_AP_CONNECTED) {
  //     // a device has connected to the AP
  //     Serial.println("Device connected to AP");
  //   } else {
  //     // a device has disconnected from the AP, and we are back in listening mode
  //     Serial.println("Device disconnected from AP");
  //   }
  // }

  static boolean gotClient = false;
  if (client) {
    if (!gotClient) {
      gotClient = true;
      sendInitials();
    }
    static char command[64] = { 0 };
    static uint8_t idx = 0;
    static bool receiving = false;
    if (client.connected()) {
      // delayMicroseconds(10);
      if (client.available()) {
        char c = client.read();
        if (c == '<') {
          receiving = true;
          command[0] = 0;
          idx = 0;
        }
        if (receiving) {
          command[idx] = c;
          command[++idx] = 0;
          if (c == '>') {
            receiving = false;
            parseCommand(command);
          }
        }
      }
      serveReturns();
      sendReturnBuffer();
    } else {
      // close the connection:
      client.stop();
    }
  } else {
    gotClient = false;
    static uint32_t lastAttempt = millis();
    if (millis() - lastAttempt >= 100) {
      client = server.available();
      lastAttempt = millis();
    }
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


  Serial.print("Creating access point named: ");
  Serial.println(ssid);  // print the network name (SSID);

  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true)
      ;
  }
  delay(2000);
  Serial.println("AP Open");
  server.begin();
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