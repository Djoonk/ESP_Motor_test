#include "esc_pwm.h"

#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "ESC_PWM";

static uint32_t pulse_us_to_duty(uint32_t pulse_us)
{
    const uint32_t max_duty = (1U << ESC_PWM_RESOLUTION) - 1U;
    const uint32_t period_us = 1000000U / ESC_PWM_FREQ_HZ;

    return (pulse_us * max_duty) / period_us;
}

esp_err_t esc_pwm_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = ESC_PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = ESC_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_channel_config_t channel_cfg = 
    {
        .gpio_num = ESC_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    esp_err_t result;

    result = ledc_timer_config(&timer_cfg);
    if (result != ESP_OK) 
        return result;
  
    result = ledc_channel_config(&channel_cfg);
    if (result != ESP_OK) 
        return result;

    esc_pwm_stop();

    ESP_LOGI(TAG, "PWM initialized: GPIO=%d, %d Hz",
             ESC_PWM_GPIO, ESC_PWM_FREQ_HZ);
    return ESP_OK;
}

void esc_pwm_set_pulse_us(uint16_t pulse_us)
{
    if (pulse_us < ESC_MIN_US)
        pulse_us = ESC_MIN_US;
    

    if (pulse_us > ESC_MAX_US) 
        pulse_us = ESC_MAX_US;
    
    uint32_t duty = pulse_us_to_duty(pulse_us);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ESP_LOGI(TAG, "PWM: %u us", pulse_us);
}

void esc_pwm_stop(void)
{
    esc_pwm_set_pulse_us(ESC_MIN_US);
    ESP_LOGI(TAG, "Motor stopped");
}

esp_err_t esc_pwm_deinit(void)
{
    esp_err_t err;

    err = ledc_stop(LEDC_LOW_SPEED_MODE,
                    LEDC_CHANNEL_0,
                    0);

    if (err != ESP_OK)
        return err;

    err = gpio_reset_pin(ESC_PWM_GPIO);

    return err;
}