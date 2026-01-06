#include "ad5940_port.h"
#include <Arduino.h>
#include <SPI.h>
#include <stdarg.h>
#include <stdio.h>

/* Pin Definitions for Arduino UNO */
#define AD5940_CS_PIN 10
#define AD5940_RESET_PIN 9
#define AD5940_INT_PIN 2

uint32_t AD5940_MCUResourceInit(void *pCfg) {
  pinMode(AD5940_CS_PIN, OUTPUT);
  pinMode(AD5940_RESET_PIN, OUTPUT);
  pinMode(AD5940_INT_PIN, INPUT_PULLUP);

  digitalWrite(AD5940_CS_PIN, HIGH);
  digitalWrite(AD5940_RESET_PIN, HIGH);

  SPI.begin();
  /* SPI settings: 250kHz, MSB First, Mode 0 */
  SPISettings settings(250000, MSBFIRST, SPI_MODE0);
  SPI.beginTransaction(settings);
  Serial.println("Debug: Port Init Done");
  Serial.flush();
  return 0;
}

void AD5940_CsClr(void) { digitalWrite(AD5940_CS_PIN, LOW); }

void AD5940_CsSet(void) { digitalWrite(AD5940_CS_PIN, HIGH); }

void AD5940_RstSet(void) { digitalWrite(AD5940_RESET_PIN, LOW); }

void AD5940_RstClr(void) { digitalWrite(AD5940_RESET_PIN, HIGH); }

void AD5940_Delay10us(uint32_t time) {
  if (time == 0)
    return;
  if (time > 1000) {
    delay(time / 100);
  } else {
    delayMicroseconds(time * 10);
  }
}

/* AD5940_ReadReg and AD5940_WriteReg are implemented in ad5940.c using
 * AD5940_ReadWriteNBytes */

void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer,
                            unsigned char *pRecvBuff, unsigned long length) {
  for (unsigned long i = 0; i < length; i++) {
    unsigned char data = 0;
    if (pSendBuffer) {
      data = pSendBuffer[i];
    }
    data = SPI.transfer(data);
    if (pRecvBuff) {
      pRecvBuff[i] = data;
    }
  }
}

uint32_t AD5940_GetMCUIntFlag(void) {
  // Check physical interrupt pin (Active Low)
  if (digitalRead(AD5940_INT_PIN) == LOW) {
    return 1;
  }
  return 0;
}

uint32_t AD5940_ClrMCUIntFlag(void) { return 1; }

void AD5940_Print(const char *fmt, ...) {
  // Simplified print to save stack.
  Serial.print(fmt);
  Serial.flush();
}

void AD5940_PrintFloat(float val) {
  Serial.print(val, 3); // Print with 3 decimal places
  Serial.flush();
}

void AD5940_PrintInt(uint32_t val) {
  Serial.print(val);
  Serial.flush();
}
