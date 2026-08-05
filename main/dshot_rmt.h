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

// Voltage calibration for EDT. AM32 sends battery_voltage/25 (1 LSB = 0.25 V).
// If the ESC's voltage divider reads low (e.g. Flycolor F051 ships with
// VOLTAGE_DIVIDER=110 instead of ~215 and reports ~half), compensate here in
// parts-per-million: 1000000 = 1.0 (no change), 1954545 = ×1.954545.
// Measure with a multimeter and set: PPM = (real_V / reported_V) * 1000000.
#define VOLTAGE_SCALE_PPM        1000000

// Extended DShot Telemetry (EDT) - the 12-bit data field is shared between
// eRPM frames and EDT frames. See https://github.com/bird-sanctuary/extended-dshot-telemetry
typedef enum {
    DSHOT_TELEMETRY_TYPE_INVALID = 0,
    DSHOT_TELEMETRY_TYPE_eRPM,
    DSHOT_TELEMETRY_TYPE_TEMPERATURE,
    DSHOT_TELEMETRY_TYPE_VOLTAGE,
    DSHOT_TELEMETRY_TYPE_CURRENT,
    DSHOT_TELEMETRY_TYPE_DEBUG1,
    DSHOT_TELEMETRY_TYPE_DEBUG2,
    DSHOT_TELEMETRY_TYPE_DEBUG3,
    DSHOT_TELEMETRY_TYPE_STATE_EVENTS,
} dshot_telemetry_type_t;

typedef struct {
    dshot_telemetry_type_t type; // DSHOT_TELEMETRY_TYPE_INVALID if none
    uint32_t value;              // scaled: eRPM/100, °C, 0.01V, 0.01A
    uint16_t raw;                // unscaled payload: 8-bit for temp/voltage/current,
                                 // 12-bit eRPM ; phone does the scaling
} dshot_telemetry_t;

// Init RMT TX + RX (bidirectional) channels, the DShot encoder and enable
// the TX channel. Call before sending. Non-blocking.
esp_err_t dshot_rmt_init(bool bidirectional);

esp_err_t dshot_rmt_deinit(void);

// Sends one DShot frame. Non-blocking (queued on hardware), like DShotRMT::send().
// Returns true if the frame was queued, false if the transmit queue was full
// (dropped frame) or the channel is disabled.
bool dshot_rmt_send(uint16_t value, bool telemetry_req);

// Busy-waits for the telemetry response of the last dshot_rmt_send() and returns
// the eRPM (1 LSB = 100 eRPM). ESP_OK on success, ESP_ERR_TIMEOUT otherwise,
// like DShotRMT::waitForErpm(). With EDT enabled, non-eRPM frames count as
// timeout; use dshot_rmt_wait_telemetry() for full EDT.
esp_err_t dshot_rmt_wait_erpm(uint32_t *erpm);

// Busy-waits for the telemetry response of the last dshot_rmt_send() and
// decodes it as full EDT (eRPM / temperature / voltage / current / debug).
esp_err_t dshot_rmt_wait_telemetry(dshot_telemetry_t *out);

// Returns the most recent eRPM, or INVALID_TELEMETRY_VALUE. Non-blocking,
// like DShotRMT::getErpm().
uint32_t dshot_rmt_get_erpm(void);

// Returns the most recent telemetry frame decoded as full EDT. Non-blocking.
void dshot_rmt_get_telemetry(dshot_telemetry_t *out);

// Returns the raw (undecoded) 20-bit GCR value of the most recent telemetry
// reply, for debugging. 0 if none. Non-blocking.
uint32_t dshot_rmt_get_raw_gcr(void);

void dshot_rmt_set_bitrate(uint32_t bitrate_khz);

void dshot_rmt_reset_telemetry(void);