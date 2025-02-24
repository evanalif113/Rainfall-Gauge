#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <ElegantOTA.h>
#include <ArduinoJson.h>
#include "DFRobot_RainfallSensor.h"

// Konfigurasi WiFi
const char* ssid = "server";
const char* password = "jeris6467";

// Buat objek WebServer di port 80
WebServer server(80);

// Inisialisasi sensor curah hujan (DFRobot_RainfallSensor)
// Pastikan sensor tersambung melalui I2C (SDA dan SCL ESP32 default: GPIO21 dan GPIO22)
DFRobot_RainfallSensor_I2C Sensor(&Wire);

// Variabel global untuk data sensor
float sensorWorkingTime = 0;
float totalRainfall = 0;
float oneHourRainfall = 0;
int rawData = 0;
void updateSensorData() {
  // Update data sensor curah hujan setiap detik
  sensorWorkingTime = Sensor.getSensorWorkingTime() * 60;
  totalRainfall = Sensor.getRainfall();            // Total curah hujan (mm)
  oneHourRainfall = Sensor.getRainfall(1);           // Curah hujan selama 1 jam (mm)
  rawData = Sensor.getRawData();                     // Jumlah tipping bucket

  // Tampilkan data ke Serial Monitor untuk debugging
  Serial.print("Sensor WorkingTime: ");
  Serial.print(sensorWorkingTime);
  Serial.println(" Minute");
  Serial.print("Total Rainfall: ");
  Serial.print(totalRainfall);
  Serial.println(" mm");
  Serial.print("1 Hour Rainfall: ");
  Serial.print(oneHourRainfall);
  Serial.println(" mm");
  Serial.print("Raw Data: ");
  Serial.println(rawData);
}

//
// Handler untuk menyajikan file index.html dari LittleFS
//
void handleRoot() {
  File file = LittleFS.open("/index.html", "r");
  if (!file) {
    server.send(500, "text/plain", "Index file not found");
    return;
  }
  server.streamFile(file, "text/html");
  file.close();
}

//
// Handler untuk menyajikan data sensor dalam format JSON
//
void handleData() {
  JsonDocument doc;
  doc["workingTime"] = sensorWorkingTime;
  doc["totalRainfall"] = totalRainfall;
  doc["oneHourRainfall"] = oneHourRainfall;
  doc["rawData"] = rawData;

  String jsonStr;
  serializeJson(doc, jsonStr);
  server.send(200, "application/json", jsonStr);
}
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Inisialisasi LittleFS
  if (!LittleFS.begin(true)) {  // Format otomatis jika mount gagal
    Serial.println("LittleFS mount failed");
    while (1);
  }
  Serial.println("LittleFS mounted successfully");

  // Inisialisasi WiFi dalam mode STA
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected, IP: ");
  Serial.println(WiFi.localIP());

  // Inisialisasi I2C (gunakan Wire default pada ESP32: SDA=21, SCL=22)
  Wire.begin();

  // Inisialisasi sensor curah hujan
  while (!Sensor.begin()) {
    Serial.println("Sensor init err!!!");
    delay(1000);
  }
  Serial.print("vid: ");
  Serial.println(Sensor.vid, HEX);
  Serial.print("pid: ");
  Serial.println(Sensor.pid, HEX);
  Serial.print("Firmware Version: ");
  Serial.println(Sensor.getFirmwareVersion());
  
  // Set nilai awal curah hujan (unit: mm)
  Sensor.setRainAccumulatedValue(0.2794);

  // Konfigurasi endpoint web server
  server.on("/", handleRoot);
  server.on("/data", handleData);
  ElegantOTA.begin(&server);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();

  server.handleClient();
  ElegantOTA.loop();
  if (currentTime - lastUpdateTime >= 1000) {
    updateSensorData();
  
    lastUpdateTime = currentTime;
  }
}


