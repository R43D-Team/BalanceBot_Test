#include "R43D_WebPage.h"

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)

int status = WL_IDLE_STATUS;
WiFiServer server(2080);
IPAddress ipAddress(192, 168, 1, 81);
WiFiClient client;

extern void parseCommand(char* command);
extern void serveReturns(WiFiClient* client);

void handleClient() {

  if (client) {
    static char command[64] = { 0 };
    static uint8_t idx = 0;
    static bool receiving = false;
    if (client.connected()) {
      // delayMicroseconds(10);
      if (client.available()) {
        char c = client.read();
        if(c == '<'){
          receiving = true;
          command[0] = 0;
          idx = 0;
        }
        if(receiving){
          command[idx] = c;
          command[++idx] = 0;
          if(c == '>') {
            receiving = false;
            parseCommand(command);
          }
        }
      }
      serveReturns(&client);
    } else {
      // close the connection:
      client.stop();
    }
  } else {
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

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);  // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected");
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