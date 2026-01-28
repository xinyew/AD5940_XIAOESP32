/*!
 *****************************************************************************
 @file:    AD5940Main.h
 @brief:   Header file for AD5940Main.c - AD5940 application control interface.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef _AD5940MAIN_H_
#define _AD5940MAIN_H_

#include <stdbool.h>
#include <stdint.h>

#define SWV_VOLTAGE_STEP_BUFFER_SIZE 1024

#ifdef __cplusplus
extern "C" {
#endif

extern float VoltageStepBuffer[SWV_VOLTAGE_STEP_BUFFER_SIZE];
extern uint32_t VoltageStepCount;

/**
 * @brief Initialize the AD5940 AFE with default platform configuration.
 * @note This function will reset AD5940 using the hardware reset pin.
 */
void AD5940_Init(void);

/**
 * @brief Run the Square Wave Voltammetry measurement.
 * @note This function blocks until measurement completes.
 */
void AD5940_SWV_Main(void);

/**
 * @brief Run the Cyclic Voltammetry measurement.
 * @note This function runs measurement in a continuous loop.
 */
void AD5940_CV_Main(void);

/**
 * @brief Initialize AD5940 interrupt handling.
 */
void ad5940int_init(void);

/**
 * @brief Shut down the AD5940 for low power mode.
 * @note Call this before entering EM4 sleep mode to minimize current draw.
 *       This stops any active measurement, turns off AFE reference and LP loop,
 *       and puts the AFE into hibernate mode (~2-3uA).
 */
void AD5940_Shutdown(void);

/**
 * @brief Deinitialize SPI peripheral connected to AD5940.
 * @note Call this after AD5940_Shutdown() before entering EM4.
 */
void AD5940_SpiDeinit(void);

typedef struct {
  float RampStartVolt;
  float RampPeakVolt;
  float Frequency;
  float SqrWvAmplitude;
  float SqrWvRampIncrement;
  float SampleDelay;
  uint32_t LPTIARtiaSel;
} AppSWV_UserParams_t;

typedef struct {
  float RampStartVolt;
  float RampPeakVolt;
  uint32_t StepNumber;
  uint32_t RampDuration; // in ms
  float SampleDelay;
  uint32_t LPTIARtiaSel;
  uint32_t bRampOneDir;
} AppCV_UserParams_t;

extern AppSWV_UserParams_t AppSWV_UserParams;
extern AppCV_UserParams_t AppCV_UserParams;
extern bool AppSWV_UserParams_Valid;
extern bool AppCV_UserParams_Valid;

#ifdef __cplusplus
}
#endif

#endif /* _AD5940MAIN_H_ */
