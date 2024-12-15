#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>

// **BME280-Konfiguration**
#define BME280_I2C_ADDRESS 0x77
Adafruit_BME280 bme;

// **BNO085-Konfiguration**
Adafruit_BNO08x bno08x;
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr, offset;

sh2_SensorValue_t sensorValue;
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000; // 5 ms = 200 Hz
unsigned long calibrationStart = 0;
bool calibrationDone = false;

// **GPS-Konfiguration**
Adafruit_GPS GPS(&Wire);
#define GPSECHO false
int timezoneOffset = 1; // MEZ = 1 Stunde, MESZ = 2 Stunden
uint32_t gpsTimer = millis();

// **Funktionen**
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

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

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Initializing sensors...");

  // **I2C starten**
  Wire.begin(8, 9);

  // **BME280 initialisieren**
  if (!bme.begin(BME280_I2C_ADDRESS)) {
    Serial.println("BME280 konnte nicht gefunden werden! Prüfen Sie die Verbindung.");
    while (1);
  }
  Serial.println("BME280 erfolgreich initialisiert.");

  // **BNO085 initialisieren**
  if (!bno08x.begin_I2C()) {
    Serial.println("BNO085 konnte nicht gefunden werden! Prüfen Sie die Verbindung.");
    while (1);
  }
  Serial.println("BNO085 erfolgreich initialisiert.");
  setReports(reportType, reportIntervalUs);
  calibrationStart = millis();

  // **GPS initialisieren**
  GPS.begin(0x10); // I2C-Adresse des GPS-Moduls
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPS.println(PMTK_Q_RELEASE);
}

void loop() {
  // **BME280-Daten lesen**
  Serial.print("Temperatur: ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Luftfeuchtigkeit: ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.print("Druck: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  // **BNO085-Daten lesen**
  if (!calibrationDone && millis() - calibrationStart > 10000) {
    offset.yaw = ypr.yaw;
    offset.pitch = ypr.pitch;
    offset.roll = ypr.roll;
    calibrationDone = true;
    Serial.println("Kalibrierung abgeschlossen.");
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
    }

    float calibratedYaw = ypr.yaw - offset.yaw;
    float calibratedPitch = ypr.pitch - offset.pitch;
    float calibratedRoll = ypr.roll - offset.roll;

    Serial.print("Yaw: "); Serial.print(calibratedYaw); Serial.print("°, ");
    Serial.print("Pitch: "); Serial.print(calibratedPitch); Serial.print("°, ");
    Serial.print("Roll: "); Serial.println(calibratedRoll);
  }

  // **GPS-Daten lesen**
  char c = GPS.read();
  if (GPSECHO && c) Serial.print(c);

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }

  if (millis() - gpsTimer > 2000) {
    gpsTimer = millis();

    int localHour = GPS.hour + timezoneOffset;
    if (localHour >= 24) localHour -= 24;
    if (localHour < 0) localHour += 24;

    Serial.print("Time: ");
    if (localHour < 10) Serial.print('0');
    Serial.print(localHour); Serial.print(":");
    if (GPS.minute < 10) Serial.print('0');
    Serial.print(GPS.minute); Serial.print(":");
    if (GPS.seconds < 10) Serial.print('0');
    Serial.print(GPS.seconds);

    Serial.print(" | Date: ");
    Serial.print(GPS.day); Serial.print("/");
    Serial.print(GPS.month); Serial.print("/20");
    Serial.print(GPS.year);

    Serial.print(" | Fix: ");
    Serial.print((int)GPS.fix);
    Serial.print(" | Satellites: ");
    Serial.print((int)GPS.satellites);

    if (GPS.fix) {
      Serial.print(" | Lat: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(" | Lon: ");
      Serial.print(GPS.longitude, 4); Serial.print(GPS.lon);
      Serial.print(" | Alt: ");
      Serial.print(GPS.altitude); Serial.print("m");
      Serial.print(" | Speed: ");
      Serial.print(GPS.speed * 0.514444);
      Serial.print("m/s");
    } else {
      Serial.print(" | No GPS Fix");
    }

    Serial.println();
  }

  delay(2000); // Lesepause
}
