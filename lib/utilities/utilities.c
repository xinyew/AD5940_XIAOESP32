#include "utilities.h"
#include <stdarg.h>
#include <stdio.h>

static uint8_t g_log_target_mask = LOG_TARGET_ALL; // Default to UART & BLE

void set_log_target(uint8_t target_mask)
{
    g_log_target_mask = target_mask;
}

uint8_t get_log_target(void)
{
    return g_log_target_mask;
}

void thor_printf(const char *format, ...)
{
    char buffer[256];
    va_list args;
    
    // Check if any logging is enabled before formatting
    if (g_log_target_mask == LOG_TARGET_NONE) {
        return;
    }

    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0) {
        // UART Output
        if (g_log_target_mask & LOG_TARGET_SWO) {
            printf("%s", buffer);
        }

        // TODO: BLE Output
        if (g_log_target_mask & LOG_TARGET_BLE) {
            // ble_transmit_buffer((const uint8_t *)buffer, (uint16_t)len);
        }
    }
}
