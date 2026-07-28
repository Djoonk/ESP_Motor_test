#ifndef DSHOT_RMT_H
#define DSHOT_RMT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

esp_err_t dshot_rmt_init(bool bidirectional);
esp_err_t dshot_rmt_deinit(void);

esp_err_t dshot_rmt_send(uint16_t packet);

void dshot_rmt_set_bitrate(uint32_t bitrate);

bool dshot_rmt_get_erpm(uint16_t *erpm);

void dshot_rmt_encode_packet(uint16_t packet);

#endif