#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <math.h>
#include <stdint.h>

// Logging targets
#define LOG_TARGET_NONE 0x00
#define LOG_TARGET_SWO 0x01
#define LOG_TARGET_BLE 0x02
#define LOG_TARGET_ALL 0xFF

// Function prototypes
void thor_printf(const char *format, ...);
void set_log_target(uint8_t target_mask);
uint8_t get_log_target(void);

#endif /* UTILITIES_H_ */
