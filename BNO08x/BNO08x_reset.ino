#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// I2C-Verbindung
Adafruit_BNO08x bno08x;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr, offset;

sh2_SensorValue_t sensorValue;

// Berichtstyp und Intervall
#ifdef FAST_MODE
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000; // 2 ms = 500 Hz
#else
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000; // 5 ms = 200 Hz
#endif

// Timer für die Offset-Berechnung
unsigned long calibrationStart = 0;
bool calibrationDone = false;

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

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Adafruit BNO08x test!");

  // Initialisiere BNO085
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  // Berichte aktivieren
  setReports(reportType, reportIntervalUs);

  // Starte den Timer für die Kalibrierung
  calibrationStart = millis();
}

void loop() {
  // Kalibrierung nach 10 Sekunden durchführen
  if (!calibrationDone && millis() - calibrationStart > 10000) {
    // Aktuelle Werte als Offset speichern
    offset.yaw = ypr.yaw;
    offset.pitch = ypr.pitch;
    offset.roll = ypr.roll;
    calibrationDone = true;
    Serial.println("Calibration completed. Offset values:");
    Serial.print("Yaw offset: "); Serial.println(offset.yaw);
    Serial.print("Pitch offset: "); Serial.println(offset.pitch);
    Serial.print("Roll offset: "); Serial.println(offset.roll);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    // Daten basierend auf Berichtstyp verarbeiten
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }

    // Kalibrierte Werte berechnen
    float calibratedYaw = ypr.yaw - offset.yaw;
    float calibratedPitch = ypr.pitch - offset.pitch;
    float calibratedRoll = ypr.roll - offset.roll;

    // Ausgabe der kalibrierten Werte
    Serial.print("Yaw: "); Serial.print(calibratedYaw); Serial.print("°, ");
    Serial.print("Pitch: "); Serial.print(calibratedPitch); Serial.print("°, ");
    Serial.print("Roll: "); Serial.println(calibratedRoll);

  }
}
