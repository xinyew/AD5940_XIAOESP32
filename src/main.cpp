#include "AD5940Main.h"
#include <Arduino.h>
#include <SPI.h>
#include <ad5940.h>
#include <ble_service.h>

void setup() {
  delay(3000);
  Serial.begin(115200);
  Serial.println("Starting ESP32 AD5940...");
  Serial.flush();

  AD5940_MCUResourceInit(NULL);
  AD5940_Init();

  AD5940_SWV_Main(); // Removed: Wait for BLE Trigger
  // Initialize BLE
  BLEPort_Init();
}

void loop() { BLEPort_Loop(); }
