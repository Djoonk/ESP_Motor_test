#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t bluetooth_spp_init(void);
void bluetooth_spp_send(const uint8_t *data, size_t length);