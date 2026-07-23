#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    STAND_DISARMED,
    STAND_ARMED,
    STAND_RUNNING,
    STAND_EMERGENCY_STOP,
    STAND_ERROR
} stand_state_t;

esp_err_t esc_controller_init(void);

void esc_controller_arm(void);
void esc_controller_disarm(void);
void esc_controller_stop(void);
void esc_controller_emergency_stop(void);
void esc_controller_reset_emergency_stop(void);
void esc_controller_set_throttle(uint16_t percent);
void esc_controller_check_timeout(void);

stand_state_t esc_controller_get_state(void);
const char *esc_controller_state_to_string(stand_state_t state);