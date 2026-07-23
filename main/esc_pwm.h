#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/ledc.h"

#define ESC_PWM_GPIO        18
#define ESC_PWM_FREQ_HZ     50
#define ESC_PWM_RESOLUTION  LEDC_TIMER_16_BIT

#define ESC_MIN_US          1000
#define ESC_MAX_US          2000

esp_err_t esc_pwm_init(void);
void esc_pwm_set_pulse_us(uint16_t pulse_us);
void esc_pwm_stop(void);
esp_err_t esc_pwm_deinit(void);