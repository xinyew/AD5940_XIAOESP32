#include <Arduino.h>
#include <SPI.h>
#include "./src/ad5940lib/ad5940.h"

// Include the C library headers.
// Note: Since we moved them to src/, we can include them relative to src or
// using specific paths. But since they are now properly guarded with extern
// "C", we can include them safely. However, AD5940Main.c is not a header, it's
// a source file. We should not include .c files. Instead, we declare the
// function prototype we need.

extern "C" {
void AD5940_Main(void);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32 AD5940...");
  Serial.flush();

  AD5940_MCUResourceInit(NULL);
  AD5940_Main();
}

void loop() {
}
