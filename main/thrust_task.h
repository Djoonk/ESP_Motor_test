#pragma once

#include "esp_err.h"

// Start the standalone thrust-measurement task (HX711 load cell).
esp_err_t thrust_task_start(void);

// Get the latest measured thrust in grams (0.0f if not yet available).
float thrust_task_get_grams(void);
