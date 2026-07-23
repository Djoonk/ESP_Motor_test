#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "stdbool.h"

typedef enum
{
    DSHOT_MODE_300 = 0,
    DSHOT_MODE_600
} dshot_mode_t;

esp_err_t esc_dshot_init(dshot_mode_t mode);
void esc_dshot_deinit(void);

void esc_dshot_set_mode(dshot_mode_t mode);

// Запускає/зупиняє безперервну передачу кадрів (1 кГц).
// Викликається протокольним рівнем при перемиканні режиму.
void esc_dshot_start_stream(void);
void esc_dshot_stop_stream(void);

void esc_dshot_set_throttle(uint16_t throttle);
void esc_dshot_stop(void);
void esc_dshot_send_command(uint16_t command, bool telemetry);
uint16_t esc_dshot_make_packet(uint16_t throttle, bool telemetry);
