#include "BLEPort.h"
#include "AD5940Main.h" // For AD5940_SWV_Main prototype if needed
#include "esp_mac.h"    // Added for esp_efuse_mac_get_default
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// UUIDs from Python scripts and SiLabs project
#define SERVICE_UUID "4EFF7359-189E-4575-928C-CD7DFA477CA1"
#define LOG_CHAR_UUID "513eb430-89eb-4d7f-880d-7ee23aa0b593"
#define MEAS_CHAR_UUID "dfe54d26-a9d5-4398-acf5-2585b41dd956"

char device_name[32] = "THOR-ESP32";

BLEServer *pServer = NULL;
BLECharacteristic *pLogCharacteristic = NULL;
BLECharacteristic *pMeasCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Global flag to trigger measurement from main loop
volatile bool g_MeasureRequest = false;

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
    g_MeasureRequest = true;

    // Optional: Update value to indicate acknowledgement
    // uint8_t val = 1;
    // pCharacteristic->setValue(&val, 1);
  }

  void onWrite(BLECharacteristic *pCharacteristic) {
    // Optional: Handle write requests if needed
  }
};

void BLEPort_Init(void) {
  // Create Device Name with MAC suffix to match "THOR-xxxx" style
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  snprintf(device_name, sizeof(device_name), "THOR-%02x%02x", mac[4], mac[5]);

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
    Serial.println("Starting SWV Measurement...");

    // Call the C function from AD5940Main.c
    // Ensure this function is declared extern "C" in its header or here
    AD5940_SWV_Main();

    Serial.println("Measurement Complete.");
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
