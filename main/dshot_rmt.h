#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/rmt_encoder.h"

esp_err_t dshot_rmt_init(void);
esp_err_t dshot_rmt_send(uint16_t packet);
esp_err_t dshot_rmt_deinit(void);
void dshot_rmt_encode_packet(uint16_t packet,
                             rmt_symbol_word_t symbols[16]);
void dshot_rmt_set_bitrate(uint32_t bitrate_khz);