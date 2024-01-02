#include "R43D_WebPage.h"

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)

int status = WL_IDLE_STATUS;
WiFiServer server(2080);
IPAddress ipAddress(192, 168, 1, 81);
WiFiClient client;

extern double Input;
extern bool imuCalibrated;

void handleClient() {

  if (client) {
    static char currentLine[64] = { 0 };
    static uint8_t idx = 0;
    static bool finished = false;
    if (client.connected() && !finished) {
      // delayMicroseconds(10);
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
            client.print("<p>IMU Calibrated: ");
            client.print(imuCalibrated? "True" :"False");
            client.print("</p>");
            client.print("<p>Current Pitch: ");
            client.print(Input);
            client.print("</p>");

            // Show a list of forms
            FormUpdatable::listForms(&client);

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            finished = true;
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
    } else {
      // close the connection:
      client.stop();
      finished = false;
    }
  } else {
    client = server.available();
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