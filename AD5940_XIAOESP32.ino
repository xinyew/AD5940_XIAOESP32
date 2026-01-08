#include "./src/ad5940_apps/AD5940Main.h"
#include "./src/ad5940lib/ad5940.h"
#include <Arduino.h>
#include <SPI.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32 AD5940...");
  Serial.flush();

  AD5940_MCUResourceInit(NULL);
  AD5940_SWV_Main();
}

void loop() {}
