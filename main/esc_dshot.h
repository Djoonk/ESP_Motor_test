#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "stdbool.h"

typedef enum
{
    DSHOT_MODE_300 = 0,
    DSHOT_MODE_600
} dshot_mode_t;

typedef struct {
    uint16_t erpm;
    uint16_t rpm;
    uint16_t raw_value;
    bool valid;
} dshot_telemetry_t;

esp_err_t esc_dshot_init(dshot_mode_t mode);
void esc_dshot_deinit(void);
void esc_dshot_set_mode(dshot_mode_t mode);
void esc_dshot_start_stream(void);
void esc_dshot_stop_stream(void);
void esc_dshot_set_throttle(uint16_t throttle);
void esc_dshot_stop(void);
void esc_dshot_send_command(uint16_t command, bool telemetry);
uint16_t esc_dshot_make_packet(uint16_t throttle, bool telemetry);

void esc_dshot_set_bidirectional(bool enable);
bool esc_dshot_is_bidirectional(void);
void esc_dshot_set_pole_count(uint8_t poles);
dshot_telemetry_t esc_dshot_get_telemetry(void);
