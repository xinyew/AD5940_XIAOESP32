#include <Arduino.h>
#include <SPI.h>

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
  while (!Serial)
    ;
  Serial.println("Starting UNO_AD5940ELCZ Refactored...");
  Serial.flush();
  // AD5940_Main() from AD5940Main.c handles initialization and the main loop.
  // It contains a while(1) loop, so this call will not return.
  AD5940_Main();
}

void loop() {
  // Should never be reached because AD5940_Main has a while(1) loop.
}
