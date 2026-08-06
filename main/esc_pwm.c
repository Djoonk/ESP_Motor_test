#include "esc_pwm.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ESC_PWM";

static bool pwm_armed = false;
static int64_t arm_start_time_us = 0;

static uint32_t pulse_us_to_duty(uint32_t pulse_us)
{
    const uint32_t max_duty = (1U << ESC_PWM_RESOLUTION) - 1U;
    const uint32_t period_us = 1000000U / ESC_PWM_FREQ_HZ;

    return (pulse_us * max_duty) / period_us;
}

esp_err_t esc_pwm_init(void)
{
    // Explicitly reset GPIO and drive LOW before LEDC takes over.
    // Some ESCs have a pull-up on the signal line that holds the pin HIGH
    // at boot - we must override it before the ESC samples the idle state.
    gpio_reset_pin(ESC_PWM_GPIO);
    gpio_set_direction(ESC_PWM_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ESC_PWM_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = ESC_PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = ESC_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t result = ledc_timer_config(&timer_cfg);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(result));
        return result;
    }

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

    result = ledc_channel_config(&channel_cfg);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(result));
        return result;
    }

    esc_pwm_stop();

    ESP_LOGI(TAG, "PWM initialized: GPIO=%d, %d Hz, duty@stop=%lu",
             ESC_PWM_GPIO, ESC_PWM_FREQ_HZ,
             ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
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

    ESP_LOGD(TAG, "PWM: %u us", pulse_us);
}

void esc_pwm_stop(void)
{
    esc_pwm_set_pulse_us(ESC_MIN_US);
    pwm_armed = false;
    arm_start_time_us = 0;
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

bool esc_pwm_is_armed(void)
{
    if (!pwm_armed)
        return false;

    // Check if arming delay has elapsed
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_ms = (now_us - arm_start_time_us) / 1000;

    if (elapsed_ms >= ESC_PWM_ARM_MS)
        return true;

    // Still in arming phase - log periodically
    static int64_t last_log_ms = 0;
    if (elapsed_ms - last_log_ms >= 500)
    {
        ESP_LOGI(TAG, "PWM arming: %lld ms / %d ms",
                 elapsed_ms, ESC_PWM_ARM_MS);
        last_log_ms = elapsed_ms;
    }

    return false;
}

void esc_pwm_start_arm(void)
{
    esc_pwm_set_pulse_us(ESC_MIN_US);
    pwm_armed = true;
    arm_start_time_us = esp_timer_get_time();
    ESP_LOGI(TAG, "PWM arming started, hold min for %d ms", ESC_PWM_ARM_MS);
}