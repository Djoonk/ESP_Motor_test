#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// DSHOT Timings (MichelJansson DShot-ESP32RMT)
#define DSHOT_ARM_DELAY_MS       3200

// DShot protocol constants
#define DSHOT_THROTTLE_MIN       48
#define DSHOT_THROTTLE_MAX       2047
#define DSHOT_ERPM_GCR_BITS      21
#define DSHOT_TELEMETRY_GCR_BITS 110
#define INVALID_TELEMETRY_VALUE  0xffff

// Init RMT TX + RX (bidirectional) channels, the DShot encoder and enable
// the TX channel. Call before sending. Non-blocking.
esp_err_t dshot_rmt_init(bool bidirectional);

esp_err_t dshot_rmt_deinit(void);

// Sends one DShot frame. Non-blocking (queued on hardware), like DShotRMT::send().
void dshot_rmt_send(uint16_t value, bool telemetry_req);

// Busy-waits for the telemetry response of the last dshot_rmt_send() and returns
// the eRPM (1 LSB = 100 eRPM). ESP_OK on success, ESP_ERR_TIMEOUT otherwise,
// like DShotRMT::waitForErpm().
esp_err_t dshot_rmt_wait_erpm(uint32_t *erpm);

// Returns the most recent eRPM, or INVALID_TELEMETRY_VALUE. Non-blocking,
// like DShotRMT::getErpm().
uint32_t dshot_rmt_get_erpm(void);

void dshot_rmt_set_bitrate(uint32_t bitrate_khz);

void dshot_rmt_reset_telemetry(void);