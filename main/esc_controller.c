#include "esc_controller.h"
#include "esp_timer.h"
#include "esc_pwm.h"
#include "esp_log.h"
#include "esc_protocol.h"
#include "esc_dshot.h"


#define SAFE_MAX_THROTTLE_PERCENT 100 // обмеження газу
#define COMMAND_TIMEOUT_MS 1000

static const char *TAG = "ESC_CTRL";

static stand_state_t current_state = STAND_DISARMED;
static uint16_t current_throttle_percent = 0;
static int64_t last_throttle_command_us = 0;

esp_err_t esc_controller_init(void)
{
    // esp_err_t result = esc_pwm_init();

    // if (result != ESP_OK)
    // {
    //     current_state = STAND_ERROR;
    //     ESP_LOGE(TAG, "PWM initialization failed");
    //     return result;
    // }
    esc_protocol_init();
    
    current_state = STAND_DISARMED;
    current_throttle_percent = 0;

    ESP_LOGI(TAG, "ESC controller initialized: DISARMED");
    return ESP_OK;
}

void esc_controller_arm(void)
{
    if (current_state == STAND_EMERGENCY_STOP)
    {
        ESP_LOGW(TAG, "Cannot arm: emergency stop is active");
        return;
    }

    if (current_state == STAND_ERROR)
    {
        ESP_LOGW(TAG, "Cannot arm: controller error");
        return;
    }

    // For DShot: the stream is already running from protocol selection.
    // Re-arm the ESC so it accepts throttle and EDT is re-enabled.
    if (esc_protocol_is_dshot())
    {
        esc_dshot_rearm();
    }

    esc_protocol_stop();

    // Start PWM arming sequence (hold min for ESC_PWM_ARM_MS)
    if (esc_protocol_get() == ESC_PROTOCOL_PWM)
    {
        esc_pwm_start_arm();
    }

    current_throttle_percent = 0;
    current_state = STAND_ARMED;

    ESP_LOGI(TAG, "ESC armed. Maximum throttle: %u%%",
             SAFE_MAX_THROTTLE_PERCENT);
}

void esc_controller_disarm(void)
{
    esc_protocol_stop();
    current_throttle_percent = 0;
    current_state = STAND_DISARMED;

    ESP_LOGI(TAG, "ESC disarmed");
}

void esc_controller_stop(void)
{
    esc_protocol_stop();
    current_throttle_percent = 0;

    if (current_state == STAND_RUNNING)
    {
        current_state = STAND_ARMED;
    }

    ESP_LOGI(TAG, "Motor stopped. State: %s",
             esc_controller_state_to_string(current_state));
}

void esc_controller_set_throttle(uint16_t percent)
{
    if (current_state != STAND_ARMED &&
        current_state != STAND_RUNNING)
    {
        ESP_LOGW(TAG, "ESC is not armed");
        esc_protocol_stop();
        return;
    }

    // For PWM: check if arming delay has elapsed
    if (esc_protocol_get() == ESC_PROTOCOL_PWM && !esc_pwm_is_armed())
    {
        ESP_LOGW(TAG, "PWM still arming, ignoring throttle");
        return;
    }

    if (percent > SAFE_MAX_THROTTLE_PERCENT)
    {
        percent = SAFE_MAX_THROTTLE_PERCENT;
        ESP_LOGW(TAG, "Throttle limited to %u%%",
                 SAFE_MAX_THROTTLE_PERCENT);
    }

    esc_protocol_set_throttle(percent);
    current_throttle_percent = percent;
    last_throttle_command_us = esp_timer_get_time();

    if (percent == 0)
    {
        current_state = STAND_ARMED;
    }
    else
    {
        current_state = STAND_RUNNING;
    }

    ESP_LOGI(TAG, "Throttle: %u%%, state: %s",
             current_throttle_percent,
             esc_controller_state_to_string(current_state));
}

stand_state_t esc_controller_get_state(void)
{
    return current_state;
}

const char *esc_controller_state_to_string(stand_state_t state)
{
    switch (state)
    {
    case STAND_DISARMED:
        return "DISARMED";

    case STAND_ARMED:
        return "ARMED";

    case STAND_RUNNING:
        return "RUNNING";

    case STAND_EMERGENCY_STOP:
        return "EMERGENCY_STOP";

    case STAND_ERROR:
        return "ERROR";

    default:
        return "UNKNOWN";
    }
}

void esc_controller_emergency_stop(void)
{
    esc_protocol_stop();

    current_throttle_percent = 0;
    current_state = STAND_EMERGENCY_STOP;
    last_throttle_command_us = 0;

    ESP_LOGE(TAG, "EMERGENCY STOP ACTIVE");
}

void esc_controller_reset_emergency_stop(void)
{
    if (current_state != STAND_EMERGENCY_STOP)
    {
        ESP_LOGW(TAG, "Emergency stop is not active");
        return;
    }

    esc_protocol_stop();

    current_throttle_percent = 0;
    current_state = STAND_DISARMED;
    last_throttle_command_us = 0;

    ESP_LOGI(TAG, "Emergency stop reset. State: DISARMED");
}

void esc_controller_check_timeout(void)
{
    if (current_state != STAND_RUNNING)
        return;

    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_ms = (now_us - last_throttle_command_us) / 1000;

    if (elapsed_ms >= COMMAND_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "Throttle command timeout: stopping motor");
        esc_controller_stop();
    }
}