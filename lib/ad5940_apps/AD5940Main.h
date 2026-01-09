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

extern float SWV_VoltageStepBuffer[SWV_VOLTAGE_STEP_BUFFER_SIZE];
extern uint32_t SWV_VoltageStepCount;

/**
 * @brief Run the Square Wave Voltammetry measurement.
 * @note This function blocks until measurement completes.
 */
void AD5940_SWV_Main(void);

#ifdef __cplusplus
}
#endif

#endif /* _AD5940MAIN_H_ */
