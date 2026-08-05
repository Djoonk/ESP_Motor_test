#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "stdbool.h"
#include "dshot_rmt.h"

typedef enum
{
    DSHOT_MODE_300 = 0,
    DSHOT_MODE_600
} dshot_mode_t;

typedef struct {
    uint16_t erpm;          // raw 12-bit EDT value (phone decodes to eRPM)
    uint8_t  temperature;   // raw: 1 LSB = 1 °C
    uint8_t  voltage;       // raw: 1 LSB = 0.25 V
    uint8_t  current;       // raw: 1 LSB = 0.5 A
} esc_dshot_raw_telemetry_t;

esp_err_t esc_dshot_init(dshot_mode_t mode, bool is_bidir);
void esc_dshot_deinit(void);
void esc_dshot_set_mode(dshot_mode_t mode);
void esc_dshot_start_stream(void);
void esc_dshot_stop_stream(void);
void esc_dshot_set_throttle(uint16_t throttle);
void esc_dshot_stop(void);
void esc_dshot_send_command(uint16_t command, bool telemetry);
void esc_dshot_enable_edt(void);
bool esc_dshot_is_bidirectional(void);
void esc_dshot_get_raw_telemetry(esc_dshot_raw_telemetry_t *out);
