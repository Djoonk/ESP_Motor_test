#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

// ── ADC configuration ──
#define MEAS_ADC_WIDTH_BITS   ADC_BITWIDTH_12     // 0..4095
#define MEAS_ADC_ATTEN        ADC_ATTEN_DB_12     // ~0..3.3 V range
#define MEAS_SAMPLE_COUNT     32U                 // averaging samples

// ── Current: ACHS-7125 (hall sensor) on ADC1 ──
#define MEAS_CURRENT_ADC_CH    ADC_CHANNEL_6      // GPIO34 (ACHS-7125 OUT)

// ── Voltage: divider R1=100k / R2=12k on ADC1 ──
#define MEAS_VOLTAGE_ADC_CH    ADC_CHANNEL_7      // GPIO35 (divider node)
#define MEAS_DIVIDER_R1_KOHM   100.0f             // batt -> node (nominal)
#define MEAS_DIVIDER_R2_KOHM   12.0f              // node -> GND (nominal)
// Measured divider ratio (multimeter calibration, 20 V / 25 V supply):
// K_real = (2.66 - 2.13) / (25 - 20) = 0.1060
#define MEAS_DIVIDER_K         0.1060f
// Measured constant offset after ADC line-fitting calibration
// (phone showed +0.4 V on both 20 V and 25 V): V_meas = V_real + 0.4.
#define MEAS_VOLTAGE_OFFSET_V  0.4f

// ── ACHS-7125 sensitivity (this board: ±50 A, 40 mV/A) ──
#define MEAS_SENS_MV_PER_A     40.0f
#define MEAS_VZERO_MV          2500.0f            // output at 0 A @ 5 V supply

/**
 * @brief Configure ADC1 (oneshot) for current and voltage channels.
 * @return ESP_OK on success.
 */
esp_err_t esc_measurements_init(void);

/**
 * @brief Read battery voltage through the 100k/12k divider.
 * @param[out] volts  Battery voltage in volts.
 * @return ESP_OK on success.
 */
esp_err_t esc_measurements_read_voltage(float *volts);
