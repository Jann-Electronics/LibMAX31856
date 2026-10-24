#include <Arduino.h>
#include <SPI.h>

#include "MAX31856.h"

// Pins (adjust according to hardware)
constexpr int8_t PIN_SCK  = 12;
constexpr int8_t PIN_MISO = 11;
constexpr int8_t PIN_MOSI = 10;
constexpr int8_t PIN_CS   = 13;

MAX31856::Driver sensor(PIN_CS, SPI);

void setup() {
  Serial.begin(115200);

  // Wait up to 3 s for the USB serial monitor, so the board also runs without a PC attached
  while (!Serial && millis() < 3000) {
  }

  // SPI bus must be initialized externally
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  // Configuration set before begin() is applied by begin() in one go
  sensor.setType(MAX31856::Type::K);
  sensor.setConversionMode(MAX31856::ConversionMode::OneShot);
  sensor.setNoiseFilter(MAX31856::NoiseFilter::Hz50);
  sensor.setAveraging(MAX31856::Averaging::Samples1);

  // Optional fault thresholds
  sensor.setProbeFaultThresholds(-50.0f, 1000.0f);
  sensor.setColdJunctionFaultThresholds(-20, 100);

  if (!sensor.begin()) {
    Serial.println("[MAX31856] begin() failed: check wiring and power");
    while (true)
      delay(1000);
  }

  Serial.println("[MAX31856] Initialized");
}

void loop() {
  // Probe temperature, cold junction and faults from a single conversion
  MAX31856::Measurement m;
  if (!sensor.read(m)) {
    Serial.println("[MAX31856] conversion timeout");
    delay(1000);
    return;
  }

  Serial.printf("Probe: %.3f °C  -  Cold Junction: %.3f °C\n", m.temperature, m.cold_junction);

  if (MAX31856::any(m.fault)) {
    char buffer[64];
    MAX31856::toString(m.fault, buffer, sizeof(buffer));
    Serial.printf("  ⚠ Faults: %s (0x%02X)\n", buffer, static_cast<uint8_t>(m.fault));

    // Individual bits can be checked too. With an open thermocouple the temperature is not valid
    if (MAX31856::has(m.fault, MAX31856::Fault::Open)) {
      Serial.println("  → thermocouple disconnected");
    }
  }

  delay(1000);
}
