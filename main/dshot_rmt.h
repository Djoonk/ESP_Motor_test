#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/rmt_encoder.h"

esp_err_t dshot_rmt_init(void);
esp_err_t dshot_rmt_send(uint16_t packet);
esp_err_t dshot_rmt_deinit(void);
void dshot_rmt_encode_packet(uint16_t packet,
                             rmt_symbol_word_t symbols[16]);
void dshot_rmt_set_bitrate(uint32_t bitrate_khz);

// ==================== RX-канал ====================//

esp_err_t dshot_rmt_init_rx(void);
void dshot_rmt_deinit_rx(void);

// Відправка + прийом (universal)
// Повертає кількість отриманих RMT-символів, 0 = timeout, <0 = помилка
int dshot_rmt_send_receive(uint16_t packet,
                           rmt_symbol_word_t *rx_buf,
                           size_t rx_buf_size,
                           uint32_t timeout_us);

// Декодування GCR 21→20 біт
// Повертає true якщо CRC валідний
bool dshot_rmt_decode_gcr(const rmt_symbol_word_t *symbols,
                           int count,
                           uint16_t *decoded_out);