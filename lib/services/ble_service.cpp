#include "ble_service.h"
#include "esp_mac.h"    // Added for esp_efuse_mac_get_default
#include <AD5940Main.h> // For AD5940_SWV_Main prototype if needed
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// UUIDs from Python scripts and SiLabs project
#define SERVICE_UUID "4EFF7359-189E-4575-928C-CD7DFA477CA1"
#define LOG_CHAR_UUID "513eb430-89eb-4d7f-880d-7ee23aa0b593"
#define MEAS_CHAR_UUID "dfe54d26-a9d5-4398-acf5-2585b41dd956"
#define RX_DATA_UUID "b36186fc-e0d3-4351-81fe-461c0aaa9588"

char device_name[32] = "THOR-EVAL";

BLEServer *pServer = NULL;
BLECharacteristic *pLogCharacteristic = NULL;
BLECharacteristic *pMeasCharacteristic = NULL;
BLECharacteristic *pReceiveCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Global flag to trigger measurement from main loop
volatile bool g_MeasureRequest = false;
// 0: None, 1: SWV, 2: CV
volatile int g_MeasureType = 0;

#define DATA_IN_BUFFER_SIZE 400
static uint8_t data_in_buffer[DATA_IN_BUFFER_SIZE];

static const uint32_t LPTIARtiaValues[] = {
    0,     200,   1000,   2000,   3000,   4000,   6000,   8000,   10000,
    12000, 16000, 20000,  24000,  30000,  32000,  40000,  48000,  64000,
    85000, 96000, 100000, 120000, 128000, 160000, 196000, 256000, 512000};

static void process_command_buffer(void) {
  // storage for command type
  uint32_t command_type = 0;
  static char feedback[512]; // Buffer for feedback message
  feedback[0] = '\0';
  int feedback_len = 0;

  if (DATA_IN_BUFFER_SIZE < sizeof(uint32_t))
    return;
  memcpy(&command_type, data_in_buffer, sizeof(uint32_t));

  Serial.printf("Processing Command Type: %u\n", command_type);

  if (command_type == 1) // SWV
  {
    // Parse SWV Parameters
    // Offset 4 bytes for command type
    uint8_t *pParams = &data_in_buffer[4];

    memcpy(&AppSWV_UserParams.RampStartVolt, pParams, sizeof(float));
    pParams += 4;
    memcpy(&AppSWV_UserParams.RampPeakVolt, pParams, sizeof(float));
    pParams += 4;
    memcpy(&AppSWV_UserParams.Frequency, pParams, sizeof(float));
    pParams += 4;
    memcpy(&AppSWV_UserParams.SqrWvAmplitude, pParams, sizeof(float));
    pParams += 4;
    memcpy(&AppSWV_UserParams.SqrWvRampIncrement, pParams, sizeof(float));
    pParams += 4;
    memcpy(&AppSWV_UserParams.SampleDelay, pParams, sizeof(float));
    pParams += 4;
    memcpy(&AppSWV_UserParams.LPTIARtiaSel, pParams, sizeof(uint32_t));

    AppSWV_UserParams_Valid = true;
    AppCV_UserParams_Valid = false; // Invalidate other

    feedback_len +=
        snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                 "SWV Params Received:\n");
    feedback_len +=
        snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                 "Param_RampStartVolt: %f\n", AppSWV_UserParams.RampStartVolt);
    feedback_len +=
        snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                 "Param_RampPeakVolt: %f\n", AppSWV_UserParams.RampPeakVolt);
    feedback_len +=
        snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                 "Param_Frequency: %f\n", AppSWV_UserParams.Frequency);
    feedback_len += snprintf(
        feedback + feedback_len, sizeof(feedback) - feedback_len,
        "Param_SqrWvAmplitude: %f\n", AppSWV_UserParams.SqrWvAmplitude);
    feedback_len += snprintf(
        feedback + feedback_len, sizeof(feedback) - feedback_len,
        "Param_SqrWvRampIncrement: %f\n", AppSWV_UserParams.SqrWvRampIncrement);
    feedback_len +=
        snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                 "Param_SampleDelay: %f\n", AppSWV_UserParams.SampleDelay);
    if (AppSWV_UserParams.LPTIARtiaSel <
        (sizeof(LPTIARtiaValues) / sizeof(uint32_t))) {
      feedback_len +=
          snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                   "Param_LPTIARtiaVal: %u\n",
                   LPTIARtiaValues[AppSWV_UserParams.LPTIARtiaSel]);
    } else {
      feedback_len +=
          snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                   "Param_LPTIARtiaVal_IndexError: %u\n",
                   AppSWV_UserParams.LPTIARtiaSel);
    }

    g_MeasureType = 1;
    g_MeasureRequest = true;
  } else if (command_type == 2) // CV
  {
    // Parse CV Parameters
    uint8_t *pParams = &data_in_buffer[4];

    memcpy(&AppCV_UserParams.RampStartVolt, pParams, sizeof(float));
    pParams += 4;
    memcpy(&AppCV_UserParams.RampPeakVolt, pParams, sizeof(float));
    pParams += 4;
    memcpy(&AppCV_UserParams.StepNumber, pParams, sizeof(uint32_t));
    pParams += 4;
    memcpy(&AppCV_UserParams.RampDuration, pParams, sizeof(uint32_t));
    pParams += 4;
    memcpy(&AppCV_UserParams.SampleDelay, pParams, sizeof(float));
    pParams += 4;
    memcpy(&AppCV_UserParams.LPTIARtiaSel, pParams, sizeof(uint32_t));
    pParams += 4;
    memcpy(&AppCV_UserParams.bRampOneDir, pParams, sizeof(uint32_t));

    AppCV_UserParams_Valid = true;
    AppSWV_UserParams_Valid = false; // Invalidate other

    feedback_len +=
        snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                 "CV Params Received:\n");
    feedback_len +=
        snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                 "Param_RampStartVolt: %f\n", AppCV_UserParams.RampStartVolt);
    feedback_len +=
        snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                 "Param_RampPeakVolt: %f\n", AppCV_UserParams.RampPeakVolt);
    feedback_len +=
        snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                 "Param_StepNumber: %u\n", AppCV_UserParams.StepNumber);
    feedback_len +=
        snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                 "Param_RampDuration: %u\n", AppCV_UserParams.RampDuration);
    feedback_len +=
        snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                 "Param_SampleDelay: %f\n", AppCV_UserParams.SampleDelay);
    if (AppCV_UserParams.LPTIARtiaSel <
        (sizeof(LPTIARtiaValues) / sizeof(uint32_t))) {
      feedback_len +=
          snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                   "Param_LPTIARtiaVal: %u\n",
                   LPTIARtiaValues[AppCV_UserParams.LPTIARtiaSel]);
    }
    feedback_len += snprintf(
        feedback + feedback_len, sizeof(feedback) - feedback_len,
        "Param_bRampOneDir: %u\n", (unsigned int)AppCV_UserParams.bRampOneDir);

    g_MeasureType = 2;
    g_MeasureRequest = true;
  } else {
    feedback_len +=
        snprintf(feedback + feedback_len, sizeof(feedback) - feedback_len,
                 "Unknown Command Type: %u\n", command_type);
  }

  // Print to Serial
  Serial.print(feedback);

  // Send Feedback via BLE Notification
  if (feedback_len > 0) {
    ble_transmit_buffer((const uint8_t *)feedback, (uint16_t)feedback_len);
  }
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("BLE Device Connected");
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("BLE Device Disconnected");
  }
};

class MyMeasCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    Serial.println("Measurement Triggered via BLE Read Request");
    // We cannot run the blocking AD5940_SWV_Main() here (watchdog will trigger)
    // Set flag for main loop
    g_MeasureType = 1; // Default to SWV on simple read
    g_MeasureRequest = true;

    // Optional: Update value to indicate acknowledgement
    // uint8_t val = 1;
    // pCharacteristic->setValue(&val, 1);
  }

  void onWrite(BLECharacteristic *pCharacteristic) {
    // Optional: Handle write requests if needed
  }
};

class MyReceiveCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0) {
      Serial.printf("Received data length: %d\n", value.length());

      if (value.length() <= DATA_IN_BUFFER_SIZE) {
        memcpy(data_in_buffer, value.c_str(), value.length());
        process_command_buffer();
      } else {
        Serial.println("Error: Received data exceeds buffer size");
      }
    }
  }
};

void BLEPort_Init(void) {
  // Create Device Name with MAC suffix to match "THOR-xxxx" style
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  // snprintf(device_name, sizeof(device_name), "THOR-%02x%02x", mac[4],
  // mac[5]);

  BLEDevice::init(device_name);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Log Characteristic (Notification)
  pLogCharacteristic = pService->createCharacteristic(
      LOG_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pLogCharacteristic->addDescriptor(new BLE2902());

  // Measurement Characteristic (Read triggers action)
  pMeasCharacteristic = pService->createCharacteristic(
      MEAS_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pMeasCharacteristic->setCallbacks(new MyMeasCallbacks());

  // Receive Data Characteristic (Write triggers action)
  pReceiveCharacteristic = pService->createCharacteristic(
      RX_DATA_UUID, BLECharacteristic::PROPERTY_WRITE);
  pReceiveCharacteristic->setCallbacks(new MyReceiveCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(
      0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.print("BLE Initialized. Device Name: ");
  Serial.println(device_name);
}

void BLEPort_Loop(void) {
  if (g_MeasureRequest) {
    g_MeasureRequest = false;
    Serial.println("Starting Measurement...");

    if (g_MeasureType == 1) {
      Serial.println("Executing SWV Main...");
      AD5940_SWV_Main();
    } else if (g_MeasureType == 2) {
      Serial.println("Executing CV Main...");
      AD5940_CV_Main();
    } else {
      Serial.println("Unknown Measurement Type");
    }

    Serial.println("Measurement Complete.");
    g_MeasureType = 0; // Reset
  }

  // Disconnection handling to restart advertising
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // Connection handling
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}

// C-compatible implementation
void ble_transmit_buffer(const uint8_t *buffer, uint16_t len) {
  if (deviceConnected && pLogCharacteristic) {
    // ESP32 BLE limit is typically MTU size (default 23, max 517)
    // The library handles large notifications but it's safer to chunk if
    // massive. The caller (TransmitSWVData) seems to chunk at ~240 bytes which
    // is fine if MTU is negotiated up. Note: setValue copies the data.
    pLogCharacteristic->setValue((uint8_t *)buffer, len);
    pLogCharacteristic->notify();
    // Small delay to prevent congestion if rapid fire
    delay(5);
  }
}
