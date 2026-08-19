// #include <stdio.h>
// #include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esc_controller.h"
#include "bluetooth_spp.h"
#include "command_handler.h"
#include "esc_protocol.h"
#include "dshot_rmt.h"
#include "esc_pwm.h"
#include "esc_measurements.h"
#include "HX711.h"
#include "thrust_task.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    if(esc_measurements_init() != ESP_OK)
        ESP_LOGI(TAG, "ADC init fail");

    HX711_init(eGAIN_128);

    if (HX711_zero() != ESP_OK)
        ESP_LOGI(TAG, "HX711 tare fail");

    // Calibration: 450 g reference (power bank) -> ~111300 raw counts above tare
    // => SCALE = 111300 / 450 ≈ 247 counts per gram.
    HX711_set_scale(247.0f);
        
    // NVS must be ready before esc_controller_init() so the motor config
    // (pole pairs) can be loaded.
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs);

    dshot_rmt_debug_pin_init();
    ESP_LOGI(TAG, "RMT sequence test start");
    ESP_ERROR_CHECK(esc_controller_init());
    // PWM вже ініціалізовано в esc_controller_init() → esc_protocol_init()
    // Повторний виклик скидає GPIO через gpio_reset_pin()

    if (thrust_task_start() != ESP_OK)
        ESP_LOGI(TAG, "Thrust task start fail");

    esp_err_t bt_result = bluetooth_spp_init();

    if (bt_result != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluetooth failed to start: %s",
                 esp_err_to_name(bt_result));
    }

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}