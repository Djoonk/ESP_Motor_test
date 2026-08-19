#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

// ============================================================================
// # esc_measurements — драйвер вимірювань (напруга + струм)
// ============================================================================
//
// ## Призначення
// Аналоговий замір напруги батареї та струму двигуна через ADC1 ESP32.
// Значення передаються на телефон по Bluetooth (зарезервовані байти
// телеметрії `batt_voltage` / `batt_current`).
//
// ## Апаратне підключення
// | Канал      | GPIO | Джерело                          |
// |------------|------|----------------------------------|
// | Напруга    | 35   | дільник R1=100k / R2=12k (K≈0.106) |
// | Струм      | 34   | ACHS-7125 (±50A) через дільник 1.5k/10k |
// | HX711 SCK  | 22   | тензодатчик (тяга)                     |
// | HX711 DOUT | 23   | тензодатчик (тяга)                     |
//
// ## Калібрування (мультиметр)
// - `MEAS_DIVIDER_K`            — фактичний коефіцієнт дільника напруги.
// - `MEAS_VOLTAGE_OFFSET_V`     — постійний зсув напруги.
// - `MEAS_VZERO_MV`             — вихід датчика струму при 0 А.
// - `MEAS_SENS_MV_PER_A`        — чутливість датчика (мВ/А).
// - `MEAS_CURRENT_ADC_OFFSET_MV`— зсув каналу ADC (канал читає вище реального).
//
// ## Шумозахист
// - Усереднення `MEAS_CURRENT_SAMPLES` (256) семплів на вимір.
// - EMA-фільтр (`MEAS_EMA_ALPHA`) між вимірами для згладжування.
// - Клепінг від'ємного струму до 0 (стенд тяги — односторонній струм).
//
// ## Використання
// ```c
// esc_measurements_init();               // один раз при старті
// esc_measurements_read_voltage(&v);     // V (з урахуванням дільника)
// esc_measurements_read_current(&a);     // A (з урахуванням калібровки)
// ```
// ============================================================================

// ── ADC configuration ──
#define MEAS_ADC_WIDTH_BITS   ADC_BITWIDTH_12     // 0..4095
#define MEAS_ADC_ATTEN        ADC_ATTEN_DB_12     // ~0..3.3 V range
#define MEAS_SAMPLE_COUNT     32U                 // averaging samples (voltage)
#define MEAS_CURRENT_SAMPLES  256U                // averaging samples (current, noise)
#define MEAS_EMA_ALPHA        0.4f               // EMA smoothing (0..1), lower = smoother

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

// ── ACHS-7125 (this board: ±50 A) ──
// Measured on GPIO34 (after the 1.5k/10k output divider):
//   at 0 A: Vzero = 1.683 V
//   at 1 A: V = 1.711 V -> sens = (1711 - 1683)/1.0 = 28 mV/A
// ADC channel-6 reads ~+32 mV above the true pin voltage (vs multimeter).
#define MEAS_SENS_MV_PER_A           28.0f
#define MEAS_VZERO_MV                1683.0f
#define MEAS_CURRENT_ADC_OFFSET_MV   32.0f

// ── HX711 (load cell) ──
#define HX711_SCK_GPIO         GPIO_NUM_22        // clock output
#define HX711_DOUT_GPIO        GPIO_NUM_23        // data input
// #define HX711_GAIN             128                // channel A, gain 128
#define HX711_TIMEOUT_US       500000U            // max wait for data-ready (~500 ms; HX711 period is 100 ms @10 Hz, 12.5 ms @80 Hz)
// Calibration: raw counts per 1 g. Measure with a known weight and fill in.
// TEMP: set to 1.0 so thrust (raw counts above auto-tare) is sent to the
// phone immediately. Replace with the real measured counts/gram:
//   scale = (raw_with_known_load - raw_tare) / weight_g
#define HX711_SCALE_COUNTS_PER_G   1.0f
// Tare offset (raw counts at zero load), set by hx711_tare().

/**
 * @brief Configure ADC1 (oneshot) for current and voltage channels.
 * @return ESP_OK on success.
 */
esp_err_t esc_measurements_init(void);

/**
 * @brief Initialize the HX711 load-cell amplifier (bit-bang GPIO).
 * @return ESP_OK on success.
 */
// esp_err_t hx711_init(void);

// /**
//  * @brief Read one raw 24-bit sample from the HX711.
//  * @param[out] raw  Signed 24-bit count.
//  * @return ESP_OK on success, ESP_ERR_TIMEOUT if the cell is not ready.
//  */
// esp_err_t hx711_read_raw(int32_t *raw);

// /**
//  * @brief Average several raw samples (anti-noise).
//  * @param[out] avg  Averaged signed count.
//  * @return ESP_OK on success.
//  */
// esp_err_t hx711_read_raw_avg(int32_t *avg);

// /**
//  * @brief Set the current load as zero (tare).
//  * @return ESP_OK on success.
//  */
// esp_err_t hx711_tare(void);

// /**
//  * @brief Read thrust/weight in grams.
//  * @param[out] grams  Weight in grams (0 if scale uncalibrated).
//  * @return ESP_OK on success.
//  */
// esp_err_t hx711_read_grams(float *grams);

/**
 * @brief Read battery voltage through the 100k/12k divider.
 * @param[out] volts  Battery voltage in volts.
 * @return ESP_OK on success.
 */
esp_err_t esc_measurements_read_voltage(float *volts);

/**
 * @brief Read motor current from the ACHS-7125 hall sensor.
 * @param[out] amps  Motor current in amperes (positive only).
 * @return ESP_OK on success.
 */
esp_err_t esc_measurements_read_current(float *amps);
