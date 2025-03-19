#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MAX1704X.h>
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
float sensorWorkingTime = 0,
      totalRainfall = 0,
      HourRainfall = 0,
      temperature = 0,
      humidity = 0,
      pressure = 0,
      dewPoint = 0,
      voltage = 0;
// Variabel tiping bucket
int rawData = 0;

// Sensor SHT40, BMP280, MAX17048
Adafruit_SHT4x sht4;
Adafruit_BMP280 bmp;
Adafruit_MAX17048 maxWin;

// Fungsi untuk menghitung titik embun (dew point)
float calculateDewPoint(float temperature, float humidity) {
  const float a = 17.27;
  const float b = 237.7;
  float alpha = ((a * temperature) / (b + temperature)) + log(humidity / 100.0);
  return (b * alpha) / (a - alpha);
}

void updateSensorData() {
  // Update data sensor curah hujan setiap detik
  sensorWorkingTime = Sensor.getSensorWorkingTime() * 60;
  totalRainfall = Sensor.getRainfall(24);            // Total curah hujan (mm)
  HourRainfall = Sensor.getRainfall(1);           // Curah hujan selama 1 jam (mm)
  rawData = Sensor.getRawData();                     // Jumlah tipping bucket
  sensors_event_t humidityEvent, tempEvent;
  sht4.getEvent(&humidityEvent, &tempEvent);
  temperature = tempEvent.temperature;
  humidity = humidityEvent.relative_humidity;
  pressure = bmp.readPressure() / 100.0F;
  voltage = maxWin.cellVoltage();
  dewPoint = calculateDewPoint(temperature, humidity);
  // Tampilkan data ke Serial Monitor untuk debugging
    // Cetak data ke serial
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);
  Serial.print("Dew Point: ");
  Serial.println(dewPoint);
  Serial.print("Pressure: ");
  Serial.println(pressure);
  Serial.print("Voltage: ");
  Serial.println(voltage);
  Serial.print("Sensor WorkingTime: ");
  Serial.print(sensorWorkingTime);
  Serial.println(" Minute");
  Serial.print("Total Rainfall: ");
  Serial.print(totalRainfall);
  Serial.println(" mm");
  Serial.print("1 Hour Rainfall: ");
  Serial.print(HourRainfall);
  Serial.println(" mm");
  Serial.print("Raw Data: ");
  Serial.println(rawData);
}

// Fungsi untuk inisialisasi sensor
void initSensors() {
  // Inisialisasi SHT40
  if (!sht4.begin()) {
    Serial.println("Couldn't find SHT40 sensor! Check wiring.");
    while (1);
  }
  Serial.println("Found SHT40 sensor!");

  // Inisialisasi BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("Couldn't find BMP280 sensor! Check wiring.");
    while (1);
  }
  Serial.println("Found BMP280 sensor!");
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, 
                  Adafruit_BMP280::SAMPLING_X2, 
                  Adafruit_BMP280::SAMPLING_X16, 
                  Adafruit_BMP280::FILTER_X16, 
                  Adafruit_BMP280::STANDBY_MS_500);

  // Inisialisasi MAX17048
  if (!maxWin.begin()) {
    Serial.println("Couldn't find MAX17048 sensor! Check wiring.");
    while (1);
  }
  Serial.println("Found MAX17048 sensor!");
    // Inisialisasi sensor curah hujan
  while (!Sensor.begin()) {
    Serial.println("Sensor init err!!!");
    delay(1000);
    }
  // Set nilai awal curah hujan (unit: mm)
  Sensor.setRainAccumulatedValue(0.2794);
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
  doc["HourRainfall"] = HourRainfall;
  doc["rawData"] = rawData;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["pressure"] = pressure;
  doc["dewPoint"] = dewPoint;
  doc["voltage"] = voltage;

  String jsonStr;
  serializeJson(doc, jsonStr);
  server.send(200, "application/json", jsonStr);
}

void setup() {
  Serial.begin(115200);

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
  initSensors();


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
  if (currentTime - lastUpdateTime >= 5000) {
    updateSensorData();
  
    lastUpdateTime = currentTime;
  }
}


