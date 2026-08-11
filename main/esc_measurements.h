#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

// ── ADC configuration ──
#define MEAS_ADC_WIDTH_BITS   ADC_BITWIDTH_12     // 0..4095
#define MEAS_ADC_ATTEN        ADC_ATTEN_DB_11     // ~0..3.3 V range
#define MEAS_SAMPLE_COUNT     32U                 // averaging samples

// ── Current: ACHS-7125 (hall sensor) on ADC1 ──
#define MEAS_CURRENT_ADC_CH    ADC_CHANNEL_6      // GPIO34 (ACHS-7125 OUT)

// ── Voltage: divider R1=100k / R2=12k on ADC1 ──
#define MEAS_VOLTAGE_ADC_CH    ADC_CHANNEL_7      // GPIO35 (divider node)
#define MEAS_DIVIDER_R1_KOHM   100.0f             // batt -> node
#define MEAS_DIVIDER_R2_KOHM   12.0f              // node -> GND
#define MEAS_DIVIDER_K \
    (MEAS_DIVIDER_R2_KOHM / (MEAS_DIVIDER_R1_KOHM + MEAS_DIVIDER_R2_KOHM))

// ── ACHS-7125 sensitivity (this board: ±50 A, 40 mV/A) ──
#define MEAS_SENS_MV_PER_A     40.0f
#define MEAS_VZERO_MV          2500.0f            // output at 0 A @ 5 V supply

/**
 * @brief Configure ADC1 (oneshot) for current and voltage channels.
 * @return ESP_OK on success.
 */
esp_err_t esc_measurements_init(void);
