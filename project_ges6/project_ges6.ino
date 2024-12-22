// Inner clock implementiert

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BNO08x.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// I²C-Adressen der Geräte
#define GPS_I2C_ADDRESS 0x10
#define BME280_I2C_ADDRESS 0x77
#define BNO085_I2C_ADDRESS 0x4A // Adresse des BNO085

// SD-Karten SPI-Pins für ESP32-C3
#define SD_CS_PIN 7   // Chip Select (CS)
#define SD_MOSI_PIN 6 // Master Out Slave In (MOSI)
#define SD_MISO_PIN 5 // Master In Slave Out (MISO)
#define SD_SCK_PIN 4  // Serial Clock (SCK)

// Inner clock
unsigned long startMillis;    // Zeitpunkt des Programmstarts
unsigned long lastPrintTime = 0; // Zeitpunkt der letzten Sensorabfrage

// Sensor-Instanzen
Adafruit_BME280 bme;
Adafruit_GPS GPS(&Wire);
Adafruit_BNO08x bno08x;

// Struktur für Euler-Winkel (BNO085)
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr, offset;

sh2_SensorId_t reportType = SH2_ROTATION_VECTOR;
long reportIntervalUs = 50000; // Mikrosekunden

File dataFile;

void setReports() {
  if (!bno08x.enableReport(reportType, reportIntervalUs)) {
    // Serial.println("Failed to enable stabilized remote vector for BNO085.");
    while (1);
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr) {
  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sq(qr) - sq(qi) - sq(qj) + sq(qk))) * RAD_TO_DEG;
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sq(qr) + sq(qi) + sq(qj) + sq(qk))) * RAD_TO_DEG;
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sq(qr) - sq(qi) + sq(qj) + sq(qk))) * RAD_TO_DEG;
}

String formatTwoDigits(int value) {
  if (value < 10) {
    return "0" + String(value);
  } else {
    return String(value);
  }
}

void setup() {
  delay(1000);

  // Serial.begin(115200);
  Wire.begin(8, 9);


  // Interne Uhr starten
  startMillis = millis();
  
  // // GPS initialisieren
  // if (!GPS.begin(GPS_I2C_ADDRESS)) {
  //   // Serial.println("Failed to initialize GPS! Check connection.");
  //   rgbLedWrite(RGB_BUILTIN, 10, 0, 0);
  //   while (1);
  // }
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // GPS.sendCommand(PGCMD_ANTENNA);
  // delay(1000);

  // // BME280 initialisieren
  // if (!bme.begin(BME280_I2C_ADDRESS)) {
  //   // Serial.println("Failed to initialize BME280! Check connection.");
  //   rgbLedWrite(RGB_BUILTIN, 10, 0, 0);
  //   while (1);
  // }
  
  // BNO085 initialisieren
  delay(1000);
  if (!bno08x.begin_I2C(BNO085_I2C_ADDRESS)) {
    // Serial.println("Failed to initialize BNO085! Check connection.");
    rgbLedWrite(RGB_BUILTIN, 0, 0, 10);
    while (1);
  }
  delay(1000);
  setReports();

  // SD-Karte initialisieren
  // Serial.println("Initializing SD card...");
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN)) {
    // Serial.println("SD card initialization failed!");
    rgbLedWrite(RGB_BUILTIN, 10, 0, 0);
    while (1);
  }
  // Serial.println("SD card initialized.");

  // Erstelle CSV-Datei und schreibe Header
  dataFile = SD.open("/sensor_data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("elapsed_seconds,date,time,longitude,latitude,altitude,temp,humid,pressure,yaw,pitch,roll");
    dataFile.close();
  } else {
    // Serial.println("Failed to create CSV file on SD card.");
    rgbLedWrite(RGB_BUILTIN, 100, 0, 0);
    while (1);
  }

  // Serial.println("All sensors initialized successfully!");
  // rgbLedWrite(RGB_BUILTIN, 0, 0, 10); // I2C-Bus Probleme mit BNO085, da beides über GPIO 8
}

void loop() {
  // rgbLedWrite(RGB_BUILTIN, 0, 10, 0); // I2C-Bus Probleme mit BNO085, da beides über GPIO 8

  unsigned long currentMillis = millis(); // Aktueller Zeitpunkt
  unsigned long elapsedMillis = currentMillis - startMillis; // Verstrichene Zeit in Millisekunden
  unsigned long elapsedSeconds = elapsedMillis / 1000; // Verstrichene Sekunden
  // unsigned long hours = elapsedSeconds / 3600;        // Stunden
  // unsigned long minutes = (elapsedSeconds % 3600) / 60; // Minuten
  // unsigned long seconds = elapsedSeconds % 60;        // Sekunden

  // char c = GPS.read();
  // if (GPS.newNMEAreceived()) {
  //   GPS.parse(GPS.lastNMEA());
  // }

  // Ausgabe alle 0,05 Sekunden
  if (millis() - lastPrintTime >= 50) {
    lastPrintTime = millis();

    // // GPS Daten
    // String date = GPS.fix ? String(GPS.day) + "/" + String(GPS.month) + "/" + String(GPS.year) : "N/A";
    // String time = GPS.fix ? formatTwoDigits(GPS.hour) + ":" + formatTwoDigits(GPS.minute) + ":" + formatTwoDigits(GPS.seconds) : "N/A";
    // String longitude = GPS.fix ? String(GPS.longitude, 4) + String(GPS.lon) : "N/A";
    // String latitude = GPS.fix ? String(GPS.latitude, 4) + String(GPS.lat) : "N/A";
    // String altitude = GPS.fix ? String(GPS.altitude) : "N/A";

    // // BME280 Daten
    // String temp = String(bme.readTemperature());
    // String humid = String(bme.readHumidity());
    // String pressure = String(bme.readPressure() / 100.0F);

    // BNO085 Daten
    sh2_SensorValue_t sensorValue;
    String yaw = "N/A", pitch = "N/A", roll = "N/A";
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        quaternionToEuler(
          sensorValue.un.rotationVector.real,
          sensorValue.un.rotationVector.i,
          sensorValue.un.rotationVector.j,
          sensorValue.un.rotationVector.k,
          &ypr
        );

        yaw = String(ypr.yaw, 2);
        pitch = String(ypr.pitch, 2);
        roll = String(ypr.roll, 2);
      }
    }

    // CSV-Zeile erstellen
    String csvLine = String(elapsedSeconds) + "," + yaw + "," + pitch + "," + roll;

    // Daten auf SD-Karte speichern
    dataFile = SD.open("/sensor_data.csv", FILE_APPEND);
    if (dataFile) {
      dataFile.println(csvLine);
      dataFile.close();
      // Serial.println("Data written to CSV file.");
    } else {
      // Serial.println("Failed to write to CSV file.");
      rgbLedWrite(RGB_BUILTIN, 0, 0, 10);
      while(1);
    }

    // Ausgabe in Serial Monitor
    // Serial.println(csvLine);
  }
}
