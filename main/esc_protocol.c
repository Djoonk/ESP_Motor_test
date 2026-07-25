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

#define DSHOT_PROTOCOL_SWITCH_LOW_MS 300U

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

static void select_dshot(dshot_mode_t mode)
{
    if (pwm_active)
    {
        ESP_ERROR_CHECK(esc_pwm_deinit());
        pwm_active = false;
    }

    ESP_ERROR_CHECK(esc_dshot_init(mode));
    esc_dshot_set_mode(mode);
    esc_dshot_start_stream();
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
        ESP_LOGI(TAG, "DShot switch: GPIO%d LOW for %u ms",
                 ESC_PWM_GPIO, DSHOT_PROTOCOL_SWITCH_LOW_MS);

        esc_dshot_deinit();
        drive_signal_low();
        vTaskDelay(pdMS_TO_TICKS(DSHOT_PROTOCOL_SWITCH_LOW_MS));
    }
    else if (previous_protocol != ESC_PROTOCOL_PWM)
    {
        esc_dshot_stop_stream();
    }
    if (protocol != ESC_PROTOCOL_BIDIRECTIONAL_DSHOT)
        esc_dshot_set_bidirectional(false);
    currentProtocol = protocol;

    switch (protocol)
    {
    case ESC_PROTOCOL_PWM:
        ESP_LOGI(TAG, "Protocol = PWM");
        esc_dshot_deinit();
        if (!pwm_active)
        {
            esc_pwm_init();
            pwm_active = true;
        }
        break;

    case ESC_PROTOCOL_DSHOT300:
        ESP_LOGI(TAG, "Protocol = DShot300");
        select_dshot(DSHOT_MODE_300);
        break;

    case ESC_PROTOCOL_DSHOT600:
        ESP_LOGI(TAG, "Protocol = DShot600");
        select_dshot(DSHOT_MODE_600);
        break;

    case ESC_PROTOCOL_BIDIRECTIONAL_DSHOT:
        ESP_LOGI(TAG, "Protocol = Bidirectional DShot");
        esc_dshot_set_bidirectional(true);
        select_dshot(DSHOT_MODE_300);
        break;

    default:
        break;
    }
}

esc_protocol_t esc_protocol_get(void)
{
    return currentProtocol;
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
    uint16_t value = 48 + ((2000 - 48) * throttle) / 100U;
    esc_dshot_set_throttle(value);
}

void esc_protocol_stop(void)
{
    if (currentProtocol == ESC_PROTOCOL_PWM)
    {
        esc_pwm_stop();
        return;
    }

    // Раніше тут нічого не було для DSHOT300/600 - мотор
    // не зупинявся ні по stop, ні по estop, ні по disarm.
    esc_dshot_stop();
}
