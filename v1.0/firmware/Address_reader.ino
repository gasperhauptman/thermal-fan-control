#include <OneWire.h>
#include <DallasTemperature.h>

#define ONEWIRE_PIN 2 //pin temperature sensor is connected to

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature sensors(&oneWire);

void printAddress(DeviceAddress addr) {
  for (uint8_t i = 0; i < 8; i++) {
    if (addr[i] < 16) Serial.print("0");
    Serial.print(addr[i], HEX);
    if (i < 7) Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  sensors.begin();

  Serial.print("Devices found: ");
  Serial.println(sensors.getDeviceCount());

  DeviceAddress addr;

  for (uint8_t i = 0; i < sensors.getDeviceCount(); i++) {
    if (sensors.getAddress(addr, i)) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" address: ");
      printAddress(addr);
    } else {
      Serial.print("Failed to read sensor at index ");
      Serial.println(i);
    }
  }
}

void loop() {
}