#include "thrust_task.h"
#include "HX711.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "thrust_task";

#define THRUST_TASK_PERIOD_MS   200
#define THRUST_TASK_STACK       2048
#define THRUST_TASK_PRIORITY    5

static float s_latest_g = 0.0f;

static void thrust_task(void *arg)
{
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(THRUST_TASK_PERIOD_MS));

        float g = 0.0f;
        g = HX711_get_units(5);
        s_latest_g = g;
    }
}

esp_err_t thrust_task_start(void)
{
    BaseType_t task_ok = xTaskCreate(
        thrust_task, "thrust_task",
        THRUST_TASK_STACK, NULL,
        THRUST_TASK_PRIORITY, NULL);

    if (task_ok != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create thrust task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Thrust task started (period %d ms)", THRUST_TASK_PERIOD_MS);
    return ESP_OK;
}

float thrust_task_get_grams(void)
{
    return s_latest_g;
}
