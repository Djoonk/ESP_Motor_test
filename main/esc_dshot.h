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
    uint16_t erpm;          // mechanical shaft RPM (1 LSB = 1 RPM)
    uint8_t  temperature;   // raw: 1 LSB = 1 °C
    uint8_t  voltage;       // raw EDT: 1 LSB = 0.25 V
    uint8_t  current;       // raw EDT: 1 LSB = 0.5 A
    uint8_t  batt_voltage;  // ADC battery voltage: 1 LSB = 0.1 V
    uint8_t  batt_current;  // ADC battery current: 1 LSB = 0.1 A (reserved)
    uint16_t thrust;        // thrust: 1 LSB = 1 g (reserved, HX711)
} esc_dshot_raw_telemetry_t;

esp_err_t esc_dshot_init(dshot_mode_t mode, bool is_bidir);
void esc_dshot_deinit(void);
void esc_dshot_set_mode(dshot_mode_t mode);
void esc_dshot_start_stream(void);
void esc_dshot_stop_stream(void);
void esc_dshot_set_throttle(uint16_t throttle);
void esc_dshot_stop(void);
void esc_dshot_rearm(void);
void esc_dshot_send_command(uint16_t command, bool telemetry);
void esc_dshot_enable_edt(void);
bool esc_dshot_is_bidirectional(void);
void esc_dshot_get_raw_telemetry(esc_dshot_raw_telemetry_t *out);

// Motor configuration. pole_pairs = number of magnetic poles / 2.
// Used to convert eRPM -> mechanical shaft RPM. Persisted to NVS.
void esc_dshot_set_motor_pole_pairs(uint8_t pole_pairs);
uint8_t esc_dshot_get_motor_pole_pairs(void);
