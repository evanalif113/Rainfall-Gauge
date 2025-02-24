#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <WiFiClient.h>
  #include <ESP8266WebServer.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <WebServer.h>
#endif
#include <ElegantOTA.h>
#include "DFRobot_RainfallSensor.h"

const char* ssid = "server";
const char* password = "jeris6467";
 
#if defined(ESP8266)
  ESP8266WebServer server(80);
#elif defined(ESP32)
  WebServer server(80);
#endif

DFRobot_RainfallSensor_I2C Sensor(&Wire);

void setup() {
  Serial.begin(115200);

  delay(1000);
  while(!Sensor.begin()) {
    Serial.println("Sensor init err!!!");
    delay(1000);
  }
  Serial.print("vid:\t");
  Serial.println(Sensor.vid,HEX);
  Serial.print("pid:\t");
  Serial.println(Sensor.pid,HEX);
  Serial.print("Version:\t");
  Serial.println(Sensor.getFirmwareVersion());
  //Set the rain accumulated value, unit: mm
  Sensor.setRainAccumulatedValue(0.2794);
    WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
 
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
 
  server.on("/", []() {
    server.send(200, "text/plain", "Rainfall Sensor Demo");
  });
 
  ElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  static unsigned long lastMillis = 0;
  if (millis() - lastMillis >= 1000) {
    lastMillis = millis();
    //Get the sensor working time, unit: hour
    Serial.print("Sensor WorkingTime:\t");
    Serial.print(Sensor.getSensorWorkingTime());
    Serial.println(" H");
    //Get the accumulated rainfall during the sensor working time
    Serial.print("Rainfall:\t");
    Serial.println(Sensor.getRainfall());
    //Get the accumulated rainfall within 1 hour of the system (function parameter optional 1-24)
    Serial.print("1 Hour Rainfall:\t");
    Serial.print(Sensor.getRainfall(1));
    Serial.println(" mm");
    //Get the raw data, the number of tipping buckets for rainfall, unit: times
    Serial.print("rainfall raw:\t");
    Serial.println(Sensor.getRawData());
  }
  server.handleClient();
  ElegantOTA.loop();
}