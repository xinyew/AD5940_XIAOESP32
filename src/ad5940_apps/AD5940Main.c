/*!
 *****************************************************************************
 @file:    AD5940Main.c
 @author:  $Author: nxu2 $
 @brief:   Used to control specific application and process data.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/
#include "./SqrWaveVoltammetry/SqrWaveVoltammetry.h"
#include "AD5940Main.h"
#include "../utilities/utilities.h"

/**
   User could configure following parameters
**/

static const uint32_t LPTIARtiaValues[] = {
    0, 200, 1000, 2000, 3000, 4000, 6000, 8000, 10000, 12000, 16000, 20000, 
    24000, 30000, 32000, 40000, 48000, 64000, 85000, 96000, 100000, 120000, 
    128000, 160000, 196000, 256000, 512000
};

// Buffers for SWV Data
float SWV_VoltageStepBuffer[SWV_VOLTAGE_STEP_BUFFER_SIZE];
uint32_t SWV_VoltageStepCount = 0;

#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq;    /* Measured LFOSC frequency */

// TODO: Implement this function
static void TransmitSWVData(uint32_t resultCount)
{
    // AppSWVCfg_Type *pSWVCfg;
    // AppSWVGetCfg(&pSWVCfg);

    // char chunk_buffer[256]; 
    // uint32_t chunk_len = 0;
    // chunk_buffer[0] = '\0';

    // // Helper macro to append lines
    // #define APPEND_LINE(...) do { \
    //     char line[128]; \
    //     int len = snprintf(line, sizeof(line), __VA_ARGS__); \
    //     if (chunk_len + len >= 240) { \
    //         ble_transmit_buffer((const uint8_t *)chunk_buffer, (uint16_t)chunk_len); \
    //         chunk_len = 0; \
    //         chunk_buffer[0] = '\0'; \
    //     } \
    //     strcat(chunk_buffer, line); \
    //     chunk_len += len; \
    // } while(0)

    // // 1. Transmit Header
    // APPEND_LINE("Device Name: %s\n", device_name);
    // APPEND_LINE("Param_RampStartVolt: %f\n", pSWVCfg->RampStartVolt);
    // APPEND_LINE("Param_RampPeakVolt: %f\n", pSWVCfg->RampPeakVolt);
    // APPEND_LINE("Param_VzeroStart: %f\n", pSWVCfg->VzeroStart);
    // APPEND_LINE("Param_VzeroPeak: %f\n", pSWVCfg->VzeroPeak);
    // APPEND_LINE("Param_Frequency: %f\n", pSWVCfg->Frequency);
    // APPEND_LINE("Param_SqrWvAmplitude: %f\n", pSWVCfg->SqrWvAmplitude);
    // APPEND_LINE("Param_SqrWvRampIncrement: %f\n", pSWVCfg->SqrWvRampIncrement);
    // APPEND_LINE("Param_SampleDelay: %f\n", pSWVCfg->SampleDelay);
    // APPEND_LINE("Param_LPTIARtiaVal: %u\n", LPTIARtiaValues[pSWVCfg->LPTIARtiaSel]);
    // APPEND_LINE("Param_bRampOneDir: %u\n", (unsigned int)pSWVCfg->bRampOneDir);
    
    // // Transmit Rtia
    // APPEND_LINE("Rtia,%f,%f\n", pSWVCfg->RtiaValue.Magnitude, pSWVCfg->RtiaValue.Phase);

    // if (chunk_len > 0) {
    //     ble_transmit_buffer((const uint8_t *)chunk_buffer, (uint16_t)chunk_len);
    //     chunk_len = 0;
    //     chunk_buffer[0] = '\0';
    // }

    // // 2. Transmit Voltage Steps
    // APPEND_LINE("Voltage Steps:\n");
    
    // for (uint32_t i = 0; i < SWV_VoltageStepCount; i++) {
    //     APPEND_LINE("Voltage Step: %f mV\n", SWV_VoltageStepBuffer[i]);
    // }
    // if (chunk_len > 0) {
    //     ble_transmit_buffer((const uint8_t *)chunk_buffer, (uint16_t)chunk_len); // Flush remaining
    //     chunk_len = 0;
    //     chunk_buffer[0] = '\0';
    // }

    // // 3. Transmit Data Output
    // APPEND_LINE("Data Output:\n");
    
    // float *pData = (float*)AppBuff;
    // for (uint32_t i = 0; i < resultCount; i++) {
    //     APPEND_LINE("index:%lu, %.10f\n", i, pData[i]);
    // }
    // if (chunk_len > 0) {
    //     ble_transmit_buffer((const uint8_t *)chunk_buffer, (uint16_t)chunk_len); // Flush remaining
    // }
    
    // // Send finish message via BLE
    // chunk_len = snprintf(chunk_buffer, sizeof(chunk_buffer), "SqrWave Voltammetry test finished.\n");
    // ble_transmit_buffer((const uint8_t *)chunk_buffer, (uint16_t)chunk_len);
}

/**
 * @brief An example to deal with data read back from AD5940. Here we just print data to UART
 * @note UART has limited speed, it has impact when sample rate is fast. Try to print some of the data not all of them.
 * @param pData: the buffer stored data for this application. The data from FIFO has been pre-processed.
 * @param DataCount: The available data count in buffer pData.
 * @return return 0.
*/
static int32_t RampShowResult(float *pData, uint32_t DataCount)
{
  if (!DataCount) {
    printf("Show result but not data\n");
  } else {
    printf("%d data received\n", DataCount);
  }
  static uint32_t index;
  /* Print data*/
  for(int i=0;i<DataCount;i++)
  {
    printf("index:%d, %.8f\n", index++, pData[i]);
    //printf("%.3f\n",pData[i]);
    //index++;
    //i += 10;  /* Print though UART consumes too much time. */
  }
  return 0;
}

/**
 * @brief The general configuration to AD5940 like FIFO/Sequencer/Clock. 
 * @note This function will firstly reset AD5940 using reset pin.
 * @return return 0.
*/
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;  
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();    /* Call this right after AFE reset */
	
  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bTRUE;           /* We will enable FIFO after all parameters configured */
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;   /* 2kB for FIFO, The reset 4kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;   /* */
  fifo_cfg.FIFOThresh = 4;            /*  Don't care, set it by application paramter */
  AD5940_FIFOCfg(&fifo_cfg);
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 4kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Configure GPIOs */
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;  /* GPIO1 indicates AFE is in sleep state. GPIO2 indicates ADC is sampling. */
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  /* Measure LFOSC frequency */
  /**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurement accuracy. Use XTAL to get better result. */
  LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;        /* Put sequence commands from start address of SRAM */
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  printf("LFOSC Freq:%f\n", LFOSCFreq);
 // AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);         /*  */
  return 0;
}

/**
 * @brief The interface for user to change application paramters.
 * @return return 0.
*/
void AD5940_SWV_RampStructInit(void)
{
  AppSWVCfg_Type *pRampCfg;
  
  AppSWVGetCfg(&pRampCfg);
  /* Step1: configure general parmaters */
  pRampCfg->SeqStartAddr = 0x10;                /* leave 16 commands for LFOSC calibration.  */
  pRampCfg->MaxSeqLen = 512-0x10;              /* 4kB/4 = 1024  */
  pRampCfg->RcalVal = 10000.0;                  /* 10kOhm RCAL */
  pRampCfg->ADCRefVolt = 1820.0f;              /* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */
  pRampCfg->FifoThresh = 1023;                   /* Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power. */
  pRampCfg->SysClkFreq = 16000000.0f;           /* System clock is 16MHz by default */
  pRampCfg->LFOSCClkFreq = LFOSCFreq;           /* LFOSC frequency */
	pRampCfg->AdcPgaGain = ADCPGA_1P5;
	pRampCfg->ADCSinc3Osr = ADCSINC3OSR_4;
  
  /* Step 2:Configure square wave signal parameters */
  pRampCfg->RampStartVolt = 000.0f;     /* Measurement starts at 0V*/
  pRampCfg->RampPeakVolt = -400.0f;         /* Measurement finishes at -0.4V */
  pRampCfg->VzeroStart = 1300.0f;           /* Vzero is voltage on SE0 pin: 1.3V */
  pRampCfg->VzeroPeak = 1300.0f;          /* Vzero is voltage on SE0 pin: 1.3V */
  pRampCfg->Frequency = 60.0f;                 /* Frequency of square wave in Hz */
  pRampCfg->SqrWvAmplitude = 40;       /* Amplitude of square wave in mV */
  pRampCfg->SqrWvRampIncrement = -4; /* Increment in mV*/
  pRampCfg->SampleDelay = 8.1f;             /* Time between update DAC and ADC sample. Unit is ms and must be < (1/Frequency)/2 - 0.2*/
  pRampCfg->LPTIARtiaSel = LPTIARTIA_256K;      /* Maximum current decides RTIA value */
  pRampCfg->bRampOneDir = bTRUE;//bTRUE;      /* Only measure ramp in one direction */

  SWV_VoltageStepCount = 0; // Reset before any sequence generation

  printf("Device Name: %s\n", "THOR-eval");
  printf("Param_RampStartVolt: %f\n", pRampCfg->RampStartVolt);
  printf("Param_RampPeakVolt: %f\n", pRampCfg->RampPeakVolt);
  printf("Param_VzeroStart: %f\n", pRampCfg->VzeroStart);
  printf("Param_VzeroPeak: %f\n", pRampCfg->VzeroPeak);
  printf("Param_Frequency: %f\n", pRampCfg->Frequency);
  printf("Param_SqrWvAmplitude: %f\n", pRampCfg->SqrWvAmplitude);
  printf("Param_SqrWvRampIncrement: %f\n", pRampCfg->SqrWvRampIncrement);
  printf("Param_SampleDelay: %f\n", pRampCfg->SampleDelay);
  printf("Param_LPTIARtiaVal: %u\n", LPTIARtiaValues[pRampCfg->LPTIARtiaSel]);
  printf("Param_bRampOneDir: %u\n", (unsigned int)pRampCfg->bRampOneDir);
}

void AD5940_SWV_Main(void)
{
  AD5940PlatformCfg();
  uint32_t temp;
  AD5940_SWV_RampStructInit();
  
  // Use AppBuff for sequencing commands initially (scratchpad), then reuse for data
  // AppSWVInit takes a buffer for generating commands. 
  // It writes to SRAM, so we can reuse the buffer after Init.
  AppSWVInit(AppBuff, APPBUFF_SIZE);    /* Initialize RAMP application. */

  AD5940_Delay10us(100000);   /* Add a delay to allow sensor reach equilibrium befor starting the measurement */
  AppSWVCtrl(APPCTRL_START, 0);          /* Control IMP measurement to start. */

  uint32_t appBuffIndex = 0;

  while(1)
  {
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE - appBuffIndex; // Available space
      // Pass the pointer to the current available position in AppBuff
      // NOTE: AppBuff is uint32_t*, AppSWVISR expects uint32_t* for buffer
      AppSWVISR(&AppBuff[appBuffIndex], &temp);
      
      // Print new data to SWO immediately
      float *pData = (float*)AppBuff;
      static uint32_t print_index = 0;
      if (print_index == 0) printf("Data Output:\n");
      for (uint32_t i = 0; i < temp; i++) {
          printf("index:%lu, %.10f\n", print_index + i, pData[appBuffIndex + i]);
      }
      print_index += temp;

      appBuffIndex += temp; // Update current index
      
      // Check for completion (AppSWVISR stops wakeuptimer when done)
      AppSWVCfg_Type *pSWVCfg;
      AppSWVGetCfg(&pSWVCfg);
      if(appBuffIndex >= pSWVCfg->StepNumber) 
      {
          printf("SqrWave Voltammetry test finished.\n");
          print_index = 0; // Reset for next run
          break; // Done
      }
    }
  }
  
  // TODO: Transmit all accumulated data
  // TransmitSWVData(appBuffIndex);
}

