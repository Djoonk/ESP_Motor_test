#include "esc_protocol.h"
#include "esc_pwm.h"
#include "esc_dshot.h"
#include "esp_log.h"
#include "dshot_rmt.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static esc_protocol_t currentProtocol = ESC_PROTOCOL_PWM;
static const char *TAG = "ESC_PROTOCOL";
static bool pwm_active = false;

// Signal LOW hold time between protocol changes. Keeps the ESC line at ~0
// for half a second so the ESC cleanly disarms/detects the new signal.
#define PROTOCOL_SWITCH_LOW_MS      500U

static bool is_dshot_protocol(esc_protocol_t protocol)
{
    return protocol == ESC_PROTOCOL_DSHOT300 ||
           protocol == ESC_PROTOCOL_DSHOT600 ||
           protocol == ESC_PROTOCOL_BIDIRECTIONAL_DSHOT;
}

static void drive_signal_low(void)
{
    ESP_ERROR_CHECK(gpio_reset_pin(ESC_PWM_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(ESC_PWM_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(ESC_PWM_GPIO, 0));
}

static void select_dshot(dshot_mode_t mode, bool bidir)
{
    if (pwm_active)
    {
        ESP_ERROR_CHECK(esc_pwm_deinit());
        pwm_active = false;
    }

    // Hold the signal line LOW so the ESC (re)disarms and cleanly detects the
    // new DShot signal BEFORE the throttle=0 arming stream starts. Without this
    // the motor spools up for a moment when switching into DShot.
    drive_signal_low();
    vTaskDelay(pdMS_TO_TICKS(PROTOCOL_SWITCH_LOW_MS));

    ESP_ERROR_CHECK(esc_dshot_init(mode, bidir));
    esc_dshot_set_mode(mode);
    esc_dshot_stop();           // ensure throttle=0
    esc_dshot_start_stream();   // DShot idle signal visible on GPIO immediately
}

void esc_protocol_init(void)
{
    esc_protocol_select(ESC_PROTOCOL_PWM);
    // esc_protocol_select(ESC_PROTOCOL_DSHOT300); //test
}

void esc_protocol_select(esc_protocol_t protocol)
{
    const esc_protocol_t previous_protocol = currentProtocol;

    if (is_dshot_protocol(previous_protocol) &&
        is_dshot_protocol(protocol))
    {
        ESP_LOGI(TAG, "DShot switch: %d -> %d",
                 previous_protocol, protocol);
        esc_dshot_deinit();
    }
    else if (previous_protocol != ESC_PROTOCOL_PWM)
    {
        esc_dshot_stop_stream();
    }

    currentProtocol = protocol;

    switch (protocol)
    {
    case ESC_PROTOCOL_PWM:
        ESP_LOGI(TAG, "Protocol = PWM");
        if (pwm_active)
        {
            ESP_LOGI(TAG, "PWM already active, skipping init");
            break;
        }
        esc_dshot_stop_stream();
        esc_dshot_deinit();
        // Ensure GPIO is fully released from RMT before LEDC takes over
        gpio_reset_pin(ESC_PWM_GPIO);
        drive_signal_low();
        vTaskDelay(pdMS_TO_TICKS(PROTOCOL_SWITCH_LOW_MS));
        esc_pwm_init();
        pwm_active = true;
        break;

    case ESC_PROTOCOL_DSHOT300:
        ESP_LOGI(TAG, "Protocol = DShot300");
        select_dshot(DSHOT_MODE_300, false);
        break;

    case ESC_PROTOCOL_DSHOT600:
        ESP_LOGI(TAG, "Protocol = DShot600");
        select_dshot(DSHOT_MODE_600, false);
        break;

    case ESC_PROTOCOL_BIDIRECTIONAL_DSHOT:
        ESP_LOGI(TAG, "Protocol = Bidirectional DShot");
        select_dshot(DSHOT_MODE_300, true);
        break;

    default:
        break;
    }
}

esc_protocol_t esc_protocol_get(void)
{
    return currentProtocol;
}

bool esc_protocol_is_dshot(void)
{
    return is_dshot_protocol(currentProtocol);
}

void esc_protocol_set_throttle(uint16_t throttle)
{
    if (currentProtocol == ESC_PROTOCOL_PWM)
    {
        uint16_t pulse_us =
            ESC_MIN_US + ((ESC_MAX_US - ESC_MIN_US) * throttle) / 100U;
        esc_pwm_set_pulse_us(pulse_us);
        return;
    }

    // DSHOT300 / DSHOT600 / BIDIRECTIONAL_DSHOT - однакова формула
    uint16_t value = 47 + ((2000 - 47) * throttle) / 100U;
    esc_dshot_set_throttle(value);
}

void esc_protocol_stop(void)
{
    if (currentProtocol == ESC_PROTOCOL_PWM)
    {
        esc_pwm_stop();
        return;
    }

    esc_dshot_stop();
}
