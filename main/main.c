// #include <stdio.h>
// #include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "esc_controller.h"
#include "bluetooth_spp.h"
#include "command_handler.h"
#include "esc_protocol.h"
#include "dshot_rmt.h"
#include "esc_pwm.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "RMT sequence test start");
    ESP_ERROR_CHECK(esc_controller_init());
    esc_protocol_select(ESC_PROTOCOL_PWM);  

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