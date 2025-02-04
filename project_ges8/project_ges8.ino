// BNO-Werte-Resetten

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
} ypr;

// Berichtseinstellungen
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV; // Verwenden Sie SH2_ARVR_STABILIZED_RV
long reportIntervalUs = 5000; // Mikrosekunden 200Hz

// SD-Datei
File dataFile;

// BNO--Werte-Reset Variablen
float yaw_offset = 0.0;
float pitch_offset = 0.0;
float roll_offset = 0.0;
bool reset_done = false;
unsigned long reset_time = 20000; // 20.000 ms = 20 Sekunden

// Funktion zur Aktivierung der Berichte
void setReports() {
  if (!bno08x.enableReport(reportType, reportIntervalUs)) {
    Serial.println("Failed to enable ARVR Stabilized Rotation Vector for BNO085.");
    while (1);
  }
}

// Quaternion-zu-Euler-Winkel-Funktion
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
        ypr->yaw *= RAD_TO_DEG;
        ypr->pitch *= RAD_TO_DEG;
        ypr->roll *= RAD_TO_DEG;
    }
}

// Funktion zur Formatierung von zweistelligen Zahlen
String formatTwoDigits(int value) {
  if (value < 10) {
    return "0" + String(value);
  } else {
    return String(value);
  }
}

void setup() {
  rgbLedWrite(RGB_BUILTIN, 210, 0, 0); 

  Serial.begin(115200);

  // pinMode(8, INPUT);
  // pinMode(9, INPUT);

  // I²C initialisieren (GPIO 2 und 3)
  Wire.setClock(100000); // 100 kHz
  Wire.begin(2, 3);

  delay(100);
  // GPS initialisieren
  if (!GPS.begin(GPS_I2C_ADDRESS)) {
    Serial.println("Failed to initialize GPS! Check connection.");
    rgbLedWrite(RGB_BUILTIN, 10, 0, 0); 
    while (1);
  }
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(100); // notwendig

  // BME280 initialisieren
  if (!bme.begin(BME280_I2C_ADDRESS)) {
    Serial.println("Failed to initialize BME280! Check connection.");
    rgbLedWrite(RGB_BUILTIN, 110, 0, 0); 
    while (1);
  }
  
  delay(100);
  // BNO085 initialisieren
  if (!bno08x.begin_I2C(BNO085_I2C_ADDRESS)) {
    Serial.println("Failed to initialize BNO085! Check connection.");
    rgbLedWrite(RGB_BUILTIN, 210, 0, 0); 
    while (1);
  }
  setReports();

  delay(100);
  // SD-Karte initialisieren
  Serial.println("Initializing SD card...");
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    rgbLedWrite(RGB_BUILTIN, 0, 0, 210); 
    while (1);
  }
  Serial.println("SD card initialized.");

  delay(100);
  // Erstelle CSV-Datei und schreibe Header
  dataFile = SD.open("/sensor_data.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("elapsed_seconds,date,time,longitude,latitude,altitude,temp,humid,pressure,yaw,pitch,roll");
    dataFile.close();
  } else {
    Serial.println("Failed to create CSV file on SD card.");
    rgbLedWrite(RGB_BUILTIN, 0, 0, 110);
    while (1);
  }

  // Interne Uhr starten
  startMillis = millis();

  Serial.println("All sensors initialized successfully!");
  rgbLedWrite(RGB_BUILTIN, 0, 10, 0);  
}

void loop() {
  unsigned long currentMillis = millis(); // Aktueller Zeitpunkt
  unsigned long elapsedMillis = currentMillis - startMillis; // Verstrichene Zeit in Millisekunden
  unsigned long elapsedSeconds = elapsedMillis / 1000; // Verstrichene Sekunden

  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  // Ausgabe alle 0,05 Sekunden (20 Hz)
  if (millis() - lastPrintTime >= 50) {
    lastPrintTime = millis();

    // BNO-Werte-Reset nach 20 Sekunden
    if (!reset_done && elapsedMillis >= reset_time) {
      sh2_SensorValue_t sensorValue;
      if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
          quaternionToEuler(
            sensorValue.un.arvrStabilizedRV.real,
            sensorValue.un.arvrStabilizedRV.i,
            sensorValue.un.arvrStabilizedRV.j,
            sensorValue.un.arvrStabilizedRV.k,
            &ypr,
            true // Grad
          );

          yaw_offset = ypr.yaw;   // Setze das aktuelle Yaw als Offset
          pitch_offset = ypr.pitch; // Setze das aktuelle Pitch als Offset
          roll_offset = ypr.roll;   // Setze das aktuelle Roll als Offset
          reset_done = true;


          // Optional: Benutzer informieren
          // Serial.println("Yaw-Reset durchgeführt. Aktuelle Yaw wird als 0° gesetzt.");
          rgbLedWrite(RGB_BUILTIN, 0, 210, 0); 
        }
      }
    }

    // GPS Daten
    char dateBuffer[11]; // "DD/MM/YYYY"
    if (GPS.fix) {
      snprintf(dateBuffer, sizeof(dateBuffer), "%02d/%02d/%04d", GPS.day, GPS.month, GPS.year);
    } else {
      strcpy(dateBuffer, "N/A");
    }

    char timeBuffer[9]; // "HH:MM:SS"
    if (GPS.fix) {
      snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", GPS.hour, GPS.minute, GPS.seconds);
    } else {
      strcpy(timeBuffer, "N/A");
    }

    char longitudeBuffer[20];
    if (GPS.fix) {
      snprintf(longitudeBuffer, sizeof(longitudeBuffer), "%.4f%c", GPS.longitude, GPS.lon);
    } else {
      strcpy(longitudeBuffer, "N/A");
    }

    char latitudeBuffer[20];
    if (GPS.fix) {
      snprintf(latitudeBuffer, sizeof(latitudeBuffer), "%.4f%c", GPS.latitude, GPS.lat);
    } else {
      strcpy(latitudeBuffer, "N/A");
    }

    char altitudeBuffer[10];
    if (GPS.fix) {
      snprintf(altitudeBuffer, sizeof(altitudeBuffer), "%.2f", GPS.altitude);
    } else {
      strcpy(altitudeBuffer, "N/A");
    }

    // BME280 Daten
    float temp = bme.readTemperature();
    float humid = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F;

    // BNO085 Daten
    sh2_SensorValue_t sensorValue;
    char yawStr[10] = "N/A", pitchStr[10] = "N/A", rollStr[10] = "N/A";
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
        quaternionToEuler(
          sensorValue.un.arvrStabilizedRV.real,
          sensorValue.un.arvrStabilizedRV.i,
          sensorValue.un.arvrStabilizedRV.j,
          sensorValue.un.arvrStabilizedRV.k,
          &ypr,
          true // Grad
        ); 

        // Korrigierte Werte berechnen
        float corrected_yaw = -(ypr.yaw - yaw_offset); // zum invertieren ypr.yaw - yaw_offset
        float corrected_pitch = ypr.pitch - pitch_offset;
        float corrected_roll = ypr.roll - roll_offset;

        // Optional: Korrigierte Werte innerhalb eines bestimmten Bereichs halten
        if (corrected_yaw < 0) corrected_yaw += 360.0;
        if (corrected_yaw >= 360.0) corrected_yaw -= 360.0;

        if (corrected_pitch < -180.0) corrected_pitch += 360.0;
        if (corrected_pitch > 180.0) corrected_pitch -= 360.0;

        if (corrected_roll < -180.0) corrected_roll += 360.0;
        if (corrected_roll > 180.0) corrected_roll -= 360.0;

        // Formatieren der korrigierten Werte
        snprintf(yawStr, sizeof(yawStr), "%.2f", corrected_yaw);
        snprintf(pitchStr, sizeof(pitchStr), "%.2f", corrected_pitch);
        snprintf(rollStr, sizeof(rollStr), "%.2f", corrected_roll);
      }
    }

    // Erstellen der CSV-Zeile als char-Array
    char csvLine[200];
    snprintf(csvLine, sizeof(csvLine), "%lu,%s,%s,%s,%s,%s,%.2f,%.2f,%.2f,%s,%s,%s",
             elapsedSeconds,
             dateBuffer,
             timeBuffer,
             longitudeBuffer,
             latitudeBuffer,
             altitudeBuffer,
             temp,
             humid,
             pressure,
             yawStr,
             pitchStr,
             rollStr);

    // Daten auf SD-Karte speichern
    dataFile = SD.open("/sensor_data.csv", FILE_APPEND);
    if (dataFile) {
      dataFile.println(csvLine);
      dataFile.close();
      Serial.println("Data written to CSV file.");
    } else {
      Serial.println("Failed to write to CSV file.");
      rgbLedWrite(RGB_BUILTIN, 0, 0, 110);
      while(1);
    }

    // Ausgabe in Serial Monitor
    Serial.println(csvLine);
  }
}
