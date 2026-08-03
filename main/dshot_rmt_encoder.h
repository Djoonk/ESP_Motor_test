#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/rmt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t throttle;  /*!< Throttle value */
    bool telemetry_req; /*!< Telemetry request */
} dshot_rmt_throttle_t;

typedef struct {
    uint32_t resolution;    /*!< Encoder resolution, in Hz */
    uint32_t baud_rate;     /*!< DShot protocol baud rate, e.g. 300000 for DSHOT300 */
    bool bidirectional;     /*!< DShot bidirectional mode */
    uint32_t post_delay_us; /*!< Delay time after one DShot frame, in microseconds */
} dshot_rmt_encoder_config_t;

esp_err_t rmt_new_dshot_esc_encoder(const dshot_rmt_encoder_config_t *config,
                                    rmt_encoder_handle_t *ret_encoder);

#ifdef __cplusplus
}
#endif