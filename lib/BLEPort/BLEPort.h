#ifndef _BLEPORT_H_
#define _BLEPORT_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// BLE Device Name
extern char device_name[32];

// Transmission Function (exposed to C)
void ble_transmit_buffer(const uint8_t *buffer, uint16_t len);

// Initialization (called from setup())
void BLEPort_Init(void);

// Polling/Task function (called from loop())
void BLEPort_Loop(void);

#ifdef __cplusplus
}
#endif

#endif // _BLEPORT_H_
