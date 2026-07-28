#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/rmt_encoder.h"

esp_err_t dshot_rmt_init(bool is_bidir);
esp_err_t dshot_rmt_deinit(void);
esp_err_t dshot_rmt_send(uint16_t packet);
void      dshot_rmt_encode_packet(uint16_t packet, rmt_symbol_word_t symbols[16]);
void      dshot_rmt_set_bitrate(uint32_t bitrate_khz);

// Bidirectional DShot: RX-канал живе весь час у bidir-режимі,
// на кожному кадрі лише пауза/відновлення (без нових алокацій).
esp_err_t dshot_rmt_rx_pause(void);
esp_err_t dshot_rmt_rx_resume_and_arm(void);
bool      dshot_rmt_get_erpm(uint16_t *erpm_out);

void dshot_rmt_get_rx_stats(uint32_t *done_count, uint32_t *wrong_size_count);