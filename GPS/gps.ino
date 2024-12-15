#include <Adafruit_GPS.h>

// I2C-Verbindung mit GPS
Adafruit_GPS GPS(&Wire);

// Debugging: Rohe NMEA-Daten ausgeben
#define GPSECHO false

// Timer für die Ausgabe
uint32_t timer = millis();

// Zeitzonen-Offset
int timezoneOffset = 1; // 1 Stunde für MEZ (Winterzeit), 2 Stunden für MESZ (Sommerzeit)

void setup() {
  Serial.begin(115200);  // Verbindung mit dem Serial Monitor

  // GPS initialisieren
  GPS.begin(0x10);  // I2C-Adresse des GPS-Moduls
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Aktiviert RMC (Zeit/Position) und GGA (Höhe)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // Aktualisierungsrate auf 1 Hz setzen
  GPS.sendCommand(PGCMD_ANTENNA);               // Antennenstatus abfragen

  delay(1000);
  GPS.println(PMTK_Q_RELEASE); // Firmware-Version abfragen
}

void loop() {
  // GPS-Daten lesen
  char c = GPS.read();

  // Debugging: Rohe NMEA-Daten ausgeben
  if (GPSECHO && c) Serial.print(c);

  // Wenn neue NMEA-Daten verfügbar sind
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return; // Ungültige NMEA-Daten überspringen
    }
  }

  // Ausgabe alle 1 Minute
  if (millis() - timer > 2000) {
    timer = millis(); // Timer zurücksetzen

    // Zeitzonenanpassung
    int localHour = GPS.hour + timezoneOffset;
    if (localHour >= 24) localHour -= 24;
    if (localHour < 0) localHour += 24;

    // Ausgabe der GPS-Daten in einer Zeile
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
      Serial.print(GPS.speed * 0.514444); // Umrechnung von Knoten in m/s
      Serial.print("m/s");
    } else {
      Serial.print(" | No GPS Fix");
    }

    Serial.println();
  }
}
