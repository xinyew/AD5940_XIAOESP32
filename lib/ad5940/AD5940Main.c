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
#include "SqrWaveVoltammetry.h"
#include "RampTest.h"
#include "AD5940Main.h"
#include <stdio.h>
#include <stdarg.h>
#include "ble_service.h"
#include "utilities.h"
#include <string.h>

/**
   User could configure following parameters
**/
static const uint32_t LPTIARtiaValues[] = {
    0, 200, 1000, 2000, 3000, 4000, 6000, 8000, 10000, 12000, 16000, 20000, 
    24000, 30000, 32000, 40000, 48000, 64000, 85000, 96000, 100000, 120000, 
    128000, 160000, 196000, 256000, 512000
};

#define APPBUFF_SIZE 1024
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq;    /* Measured LFOSC frequency */

// Buffers for SWV Data
float VoltageStepBuffer[SWV_VOLTAGE_STEP_BUFFER_SIZE];
uint32_t VoltageStepCount = 0;

// User User Parameters
AppSWV_UserParams_t AppSWV_UserParams;
AppCV_UserParams_t AppCV_UserParams;
bool AppSWV_UserParams_Valid = false;
bool AppCV_UserParams_Valid = false;

void AD5940_Shutdown(void) {
    // Stop any active measurement and shut down the AFE
    // APPCTRL_SHUTDOWN calls AD5940_ShutDownS() internally, which:
    // - Turns off AFE reference
    // - Turns off LP loop
    // - Puts AFE into hibernate mode
    AppSWVCtrl(APPCTRL_SHUTDOWN, 0);
}

/* ---------------------------------------------------------------------------
 * Static Helper Functions to reduce duplication
 * --------------------------------------------------------------------------- */

static void AppCommon_EquilibriumDelay(float equilibriumTime) {
    uint32_t total_delay_ms = (uint32_t)equilibriumTime;
    while(total_delay_ms > 0) {
        uint32_t chunk_ms = (total_delay_ms > 100) ? 100 : total_delay_ms;
        AD5940_Delay10us(chunk_ms * 100); 
        total_delay_ms -= chunk_ms;
    }
}

static void AppCommon_AppendAndTransmit(char *chunk_buffer, uint32_t *chunk_len, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char line[128];
    int len = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    if (*chunk_len + len >= 240) {
        ble_transmit_buffer((const uint8_t *)chunk_buffer, (uint16_t)*chunk_len);
        *chunk_len = 0;
        chunk_buffer[0] = '\0';
    }
    strcat(chunk_buffer, line);
    *chunk_len += len;
}

static void AppCommon_FlushTransmit(char *chunk_buffer, uint32_t *chunk_len) {
    if (*chunk_len > 0) {
        ble_transmit_buffer((const uint8_t *)chunk_buffer, (uint16_t)*chunk_len);
        *chunk_len = 0;
        chunk_buffer[0] = '\0';
    }
}

static void AppCommon_TransmitVoltageSteps(char *chunk_buffer, uint32_t *chunk_len) {
    AppCommon_AppendAndTransmit(chunk_buffer, chunk_len, "Voltage Steps:\n");
    for (uint32_t i = 0; i < VoltageStepCount; i++) {
        AppCommon_AppendAndTransmit(chunk_buffer, chunk_len, "Voltage Step: %f mV\n", VoltageStepBuffer[i]);
    }
    AppCommon_FlushTransmit(chunk_buffer, chunk_len); // Flush to allow large step buffers
}

static void AppCommon_TransmitDataOutput(char *chunk_buffer, uint32_t *chunk_len, uint32_t *pBuff, uint32_t resultCount) {
    AppCommon_AppendAndTransmit(chunk_buffer, chunk_len, "Data Output:\n");
    float *pData = (float*)pBuff;
    for (uint32_t i = 0; i < resultCount; i++) {
        AppCommon_AppendAndTransmit(chunk_buffer, chunk_len, "index:%lu, %.10f\n", i, pData[i]);
    }
    AppCommon_FlushTransmit(chunk_buffer, chunk_len);
}

static void AppCommon_ShowResult(float *pData, uint32_t DataCount, uint32_t *pPrintIndex, uint32_t MaxStep) {
    if (!DataCount) return;
    
    if (*pPrintIndex == 0) thor_printf("Data Output:\n");

    for(uint32_t i=0; i<DataCount; i++) {
        thor_printf("index:%lu, %.10f\n", *pPrintIndex + i, pData[i]);
    }
    *pPrintIndex += DataCount;
    
    if (*pPrintIndex >= MaxStep) {
        thor_printf("SqrWave Voltammetry test finished.\n");
    }
}

/* ---------------------------------------------------------------------------
 * Application Functions
 * --------------------------------------------------------------------------- */

typedef enum {
    APP_TYPE_SWV,
    APP_TYPE_CV
} AppType_t;

/* ---------------------------------------------------------------------------
 * Application Functions
 * --------------------------------------------------------------------------- */

static void TransmitApplicationData(uint32_t resultCount, AppType_t appType)
{
    char chunk_buffer[256]; 
    uint32_t chunk_len = 0;
    chunk_buffer[0] = '\0';

    /* 1. Transmit Header */
    AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Device Name: %s\n", device_name);
    
    if (appType == APP_TYPE_SWV) {
        AppSWVCfg_Type *pSWVCfg;
        AppSWVGetCfg(&pSWVCfg);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_RampStartVolt: %f\n", pSWVCfg->RampStartVolt);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_RampPeakVolt: %f\n", pSWVCfg->RampPeakVolt);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_VzeroStart: %f\n", pSWVCfg->VzeroStart);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_VzeroPeak: %f\n", pSWVCfg->VzeroPeak);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_Frequency: %f\n", pSWVCfg->Frequency);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_SqrWvAmplitude: %f\n", pSWVCfg->SqrWvAmplitude);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_SqrWvRampIncrement: %f\n", pSWVCfg->SqrWvRampIncrement);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_SampleDelay: %f\n", pSWVCfg->SampleDelay);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_LPTIARtiaVal: %u\n", LPTIARtiaValues[pSWVCfg->LPTIARtiaSel]);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Rtia,%f,%f\n", pSWVCfg->RtiaValue.Magnitude, pSWVCfg->RtiaValue.Phase);
    } else {
        AppRAMPCfg_Type *pRAMPCfg;
        AppRAMPGetCfg(&pRAMPCfg);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_RampStartVolt: %f\n", pRAMPCfg->RampStartVolt);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_RampPeakVolt: %f\n", pRAMPCfg->RampPeakVolt);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_VzeroStart: %f\n", pRAMPCfg->VzeroStart);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_VzeroPeak: %f\n", pRAMPCfg->VzeroPeak);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_StepNumber: %u\n", pRAMPCfg->StepNumber);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_RampDuration: %u\n", pRAMPCfg->RampDuration);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_SampleDelay: %f\n", pRAMPCfg->SampleDelay);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_LPTIARtiaVal: %u\n", LPTIARtiaValues[pRAMPCfg->LPTIARtiaSel]);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Param_bRampOneDir: %u\n", (unsigned int)pRAMPCfg->bRampOneDir);
        AppCommon_AppendAndTransmit(&chunk_buffer[0], &chunk_len, "Rtia,%f,%f\n", pRAMPCfg->RtiaValue.Magnitude, pRAMPCfg->RtiaValue.Phase);
    }
    AppCommon_FlushTransmit(&chunk_buffer[0], &chunk_len);

    /* 2. Transmit Voltage Steps */
    AppCommon_TransmitVoltageSteps(&chunk_buffer[0], &chunk_len);

    /* 3. Transmit Data Output */
    AppCommon_TransmitDataOutput(&chunk_buffer[0], &chunk_len, AppBuff, resultCount);
    
    /* Send finish message via BLE */
    chunk_len = snprintf(chunk_buffer, sizeof(chunk_buffer), "SqrWave Voltammetry test finished.\n");
    ble_transmit_buffer((const uint8_t *)chunk_buffer, (uint16_t)chunk_len);
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
  thor_printf("LFOSC Freq:%f\n", LFOSCFreq);
 // AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);         /*  */
  return 0;
}
void AD5940_Init(void) {
  AD5940PlatformCfg();
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
  pRampCfg->ADCSinc2Osr = ADCSINC2OSR_22;
  
  /* Step 2:Configure square wave signal parameters */
  if (AppSWV_UserParams_Valid)
  {
      pRampCfg->RampStartVolt = AppSWV_UserParams.RampStartVolt;
      pRampCfg->RampPeakVolt = AppSWV_UserParams.RampPeakVolt;
      pRampCfg->Frequency = AppSWV_UserParams.Frequency;
      pRampCfg->SqrWvAmplitude = AppSWV_UserParams.SqrWvAmplitude;
      pRampCfg->SqrWvRampIncrement = AppSWV_UserParams.SqrWvRampIncrement;
      pRampCfg->SampleDelay = AppSWV_UserParams.SampleDelay;
      pRampCfg->LPTIARtiaSel = AppSWV_UserParams.LPTIARtiaSel;
  }
  else
  {
      pRampCfg->RampStartVolt = 0.0f;     /* Measurement starts at 0V*/
      pRampCfg->RampPeakVolt = -400.0f;         /* Measurement finishes at -0.4V */
      pRampCfg->Frequency = 60.0f;                 /* Frequency of square wave in Hz */
      pRampCfg->SqrWvAmplitude = 40;       /* Amplitude of square wave in mV */
      pRampCfg->SqrWvRampIncrement = 4; /* Increment in mV*/
      pRampCfg->SampleDelay = 8.1f;             /* Time between update DAC and ADC sample. Unit is ms and must be < (1/Frequency)/2 - 0.2*/
      pRampCfg->LPTIARtiaSel = LPTIARTIA_256K;      /* Maximum current decides RTIA value */
  }

  pRampCfg->VzeroStart = 1300.0f;           /* Vzero is voltage on SE0 pin: 1.3V */
  pRampCfg->VzeroPeak = 1300.0f;          /* Vzero is voltage on SE0 pin: 1.3V */
  pRampCfg->bRampOneDir = bTRUE;//bTRUE;      /* Only measure ramp in one direction */
  pRampCfg->EquilibriumTime = 3000.0f;        /* 3000ms default */

  VoltageStepCount = 0; // Reset before any sequence generation

  printf("Device Name: %s\n", device_name);
  printf("Param_RampStartVolt: %f\n", pRampCfg->RampStartVolt);
  printf("Param_RampPeakVolt: %f\n", pRampCfg->RampPeakVolt);
  printf("Param_VzeroStart: %f\n", pRampCfg->VzeroStart);
  printf("Param_VzeroPeak: %f\n", pRampCfg->VzeroPeak);
  printf("Param_Frequency: %f\n", pRampCfg->Frequency);
  printf("Param_SqrWvAmplitude: %f\n", pRampCfg->SqrWvAmplitude);
  printf("Param_SqrWvRampIncrement: %f\n", pRampCfg->SqrWvRampIncrement);
  printf("Param_SampleDelay: %f\n", pRampCfg->SampleDelay);
  printf("Param_LPTIARtiaVal: %u\n", LPTIARtiaValues[pRampCfg->LPTIARtiaSel]);
}


/**
 * @brief The interface for user to change application paramters.
 * @return return 0.
*/
void AD5940_CV_RampStructInit(void)
{
  AppRAMPCfg_Type *pRampCfg;

  AppRAMPGetCfg(&pRampCfg);
  /* Step1: configure general parmaters */
  pRampCfg->SeqStartAddr = 0x10;                /* leave 16 commands for LFOSC calibration.  */
  pRampCfg->MaxSeqLen = 1024-0x10;              /* 4kB/4 = 1024  */
  pRampCfg->RcalVal = 10000.0;                  /* 10kOhm RCAL */
  pRampCfg->ADCRefVolt = 1820.0f;               /* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */
  pRampCfg->FifoThresh = 480;                   /* Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power. */
  pRampCfg->SysClkFreq = 16000000.0f;           /* System clock is 16MHz by default */
  pRampCfg->LFOSCClkFreq = LFOSCFreq;   /* LFOSC frequency */
  /* Configure ramp signal parameters */
  if (AppCV_UserParams_Valid)
  {
      pRampCfg->RampStartVolt = AppCV_UserParams.RampStartVolt;
      pRampCfg->RampPeakVolt = AppCV_UserParams.RampPeakVolt;
      pRampCfg->StepNumber = AppCV_UserParams.StepNumber;
      pRampCfg->RampDuration = AppCV_UserParams.RampDuration;
      pRampCfg->SampleDelay = AppCV_UserParams.SampleDelay;
      pRampCfg->LPTIARtiaSel = AppCV_UserParams.LPTIARtiaSel;
      pRampCfg->bRampOneDir = (BoolFlag)AppCV_UserParams.bRampOneDir;
  }
  else
  {
      pRampCfg->RampStartVolt =  -300.0f;           /* -1V */
      pRampCfg->RampPeakVolt = +600.0f;           /* +1V */
      pRampCfg->StepNumber = 45;                   /* Total steps. Equals to ADC sample number */
      pRampCfg->RampDuration = 1.50*1000;            /* X * 1000, where x is total duration of ramp signal. Unit is ms. */
      pRampCfg->SampleDelay = 1.5f;                 /* 7ms. Time between update DAC and ADC sample. Unit is ms. */
      pRampCfg->LPTIARtiaSel = LPTIARTIA_128K;       /* Maximum current decides RTIA value */
      pRampCfg->bRampOneDir = bTRUE;//bTRUE;      /* Only measure ramp in one direction */
  }

  pRampCfg->VzeroStart = 1300.0f;               /* 1.3V */
  pRampCfg->VzeroPeak = 1300.0f;                /* 1.3V */
  pRampCfg->LPTIARloadSel = LPTIARLOAD_SHORT;
  pRampCfg->AdcPgaGain = ADCPGA_1P5;
  pRampCfg->EquilibriumTime = 3000.0f; /* 0ms default */

  VoltageStepCount = 0; // Reset before any sequence generation

  printf("Device Name: %s\n", device_name);
  // printf("Param_AppType: %f\n", pRampCfg->RampStartVolt);
  printf("Param_RampStartVolt: %f\n", pRampCfg->RampStartVolt);
  printf("Param_RampPeakVolt: %f\n", pRampCfg->RampPeakVolt);
  printf("Param_VzeroStart: %f\n", pRampCfg->VzeroStart);
  printf("Param_VzeroPeak: %f\n", pRampCfg->VzeroPeak);
  printf("Param_StepNumber: %u\n", pRampCfg->StepNumber);
  printf("Param_RampDuration: %u\n", pRampCfg->RampDuration);
  printf("Param_SampleDelay: %f\n", pRampCfg->SampleDelay);
  printf("Param_LPTIARtiaVal: %u\n", LPTIARtiaValues[pRampCfg->LPTIARtiaSel]);
  printf("Param_bRampOneDir: %u\n", (unsigned int)pRampCfg->bRampOneDir);
}

void AD5940_SWV_Main(void)
{
  uint32_t temp;
  AD5940_SWV_RampStructInit();
  
  // Use AppBuff for sequencing commands initially (scratchpad), then reuse for data
  AppSWVInit(AppBuff, APPBUFF_SIZE);    /* Initialize RAMP application. */

  AppSWVCfg_Type *pSWVCfg;
  AppSWVGetCfg(&pSWVCfg);
  
  AppCommon_EquilibriumDelay(pSWVCfg->EquilibriumTime);
          
  AppSWVCtrl(APPCTRL_START, 0);          /* Control IMP measurement to start. */

  uint32_t appBuffIndex = 0;
  static uint32_t print_index = 0;
  print_index = 0; // Reset

  while(1)
  {
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE - appBuffIndex; // Available space
      AppSWVISR(&AppBuff[appBuffIndex], &temp);
      
      // Print new data to SWO using helper
      AppCommon_ShowResult((float*)&AppBuff[appBuffIndex], temp, &print_index, pSWVCfg->StepNumber);

      appBuffIndex += temp; // Update current index
      
      if(appBuffIndex >= pSWVCfg->StepNumber) 
      {
          break; // Done
      }
    }
  }
  
  // Transmit all accumulated data
  TransmitApplicationData(appBuffIndex, APP_TYPE_SWV);
}


void AD5940_CV_Main(void)
{
  uint32_t temp;
  AD5940_CV_RampStructInit();

  AppRAMPInit(AppBuff, APPBUFF_SIZE);    /* Initialize RAMP application. */
  
  AppRAMPCfg_Type *pRampCfg;
  AppRAMPGetCfg(&pRampCfg);
  
  AppCommon_EquilibriumDelay(pRampCfg->EquilibriumTime);

  AppRAMPCtrl(APPCTRL_START, 0);          /* Control IMP measurement to start. */

  uint32_t appBuffIndex = 0;
  static uint32_t print_index = 0;
  print_index = 0; // Reset

  while(1)
  {
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE - appBuffIndex;
      AppRAMPISR(&AppBuff[appBuffIndex], &temp);

      // Print new data to SWO using helper
      AppCommon_ShowResult((float*)&AppBuff[appBuffIndex], temp, &print_index, pRampCfg->StepNumber);

      appBuffIndex += temp; // Update current index
      
      if(appBuffIndex >= pRampCfg->StepNumber) 
      {
          break; // Done
      }
    }
  }
  // Transmit all accumulated data
  TransmitApplicationData(appBuffIndex, APP_TYPE_CV);
}
