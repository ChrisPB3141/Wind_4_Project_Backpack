#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// I2C-Adresse des BME280
// #define BME280_I2C_ADDRESS 0x77  // Prüfen Sie ggf. mit 0x77

Adafruit_BME280 bme;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // I2C starten: Gelb = SDA (GPIO 8), Blau = SCL (GPIO 9)
  Wire.begin(8, 9);

  // BME280 initialisieren
  if (!bme.begin(BME280_I2C_ADDRESS)) {
    Serial.println("BME280 konnte nicht gefunden werden! Prüfen Sie die Verbindung.");
    while (1);
  }

  Serial.println("BME280 erfolgreich initialisiert.");
}

void loop() {
  Serial.print("Temperatur: ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Luftfeuchtigkeit: ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.print("Druck: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  delay(2000);
}
