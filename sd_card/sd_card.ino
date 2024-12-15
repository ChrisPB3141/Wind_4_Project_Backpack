#include "FS.h"
#include "SD.h"
#include "SPI.h"

void writeRandomData(fs::FS &fs, const char *path, size_t dataSize) {
  Serial.printf("Creating file with random data: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  // Schreibe Zufallszahlen als Text in die Datei
  for (size_t i = 0; i < dataSize; i++) {
    int randomNumber = random(0, 1000); // Zufällige Zahl zwischen 0 und 999
    file.print(randomNumber);          // Schreibe die Zahl als Text
    file.print(" ");                   // Leerzeichen zwischen den Zahlen
  }

  Serial.printf("Successfully wrote %zu random numbers to %s\n", dataSize, path);
  file.close();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("Initializing SD card...");

  // Initialisiere SD-Karte
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  // Schreibe Zufallszahlen in eine .txt-Datei
  writeRandomData(SD, "/random_data.txt", 100); // 100 Zufallszahlen
}

void loop() {
  // Leerer Loop
}
