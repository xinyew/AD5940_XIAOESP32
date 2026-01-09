#include "ad5940.h"
#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>

#define AD5940_SCK_PIN D0
#define AD5940_MISO_PIN D1
#define AD5940_MOSI_PIN D2
#define AD5940_CS_PIN D3
#define AD5940_RESET_PIN D4
#define AD5940_INT_PIN D5

#define AD5940_SPI_CLOCK 4000000

volatile uint32_t ucInterrupted = 0;

void IRAM_ATTR ad5940int_handler() { ucInterrupted = 1; }

uint32_t AD5940_MCUResourceInit(void *pCfg) {
  pinMode(AD5940_CS_PIN, OUTPUT);
  pinMode(AD5940_RESET_PIN, OUTPUT);
  pinMode(AD5940_INT_PIN, INPUT_PULLUP);

  digitalWrite(AD5940_CS_PIN, HIGH);
  digitalWrite(AD5940_RESET_PIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(AD5940_INT_PIN), ad5940int_handler,
                  FALLING);
  SPI.begin(AD5940_SCK_PIN, AD5940_MISO_PIN, AD5940_MOSI_PIN, -1);

  return 0;
}

void AD5940_CsClr(void) { digitalWrite(AD5940_CS_PIN, LOW); }

void AD5940_CsSet(void) { digitalWrite(AD5940_CS_PIN, HIGH); }

void AD5940_RstSet(void) { digitalWrite(AD5940_RESET_PIN, HIGH); }

void AD5940_RstClr(void) { digitalWrite(AD5940_RESET_PIN, LOW); }

void AD5940_Delay10us(uint32_t time) { delayMicroseconds(time * 10); }

void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer,
                            unsigned char *pRecvBuff, unsigned long length) {

  SPI.transferBytes(pSendBuffer, pRecvBuff, length);
}

uint32_t AD5940_GetMCUIntFlag(void) { return ucInterrupted; }

uint32_t AD5940_ClrMCUIntFlag(void) {
  ucInterrupted = 0;
  return 1;
}