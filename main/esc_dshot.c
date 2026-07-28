#include "esc_dshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dshot_rmt.h"
#include "esp_timer.h"

static const char *TAG = "ESC_DSHOT";
static uint32_t rx_arm_count = 0;

#define DSHOT_FRAME_PERIOD_US 500U
#define DSHOT_FRAME_RATE_HZ 2000U
#define DSHOT_TASK_STACK_SIZE 2048
#define DSHOT_TASK_PRIORITY 2

static dshot_mode_t currentMode = DSHOT_MODE_300;
static bool bidir_active = false;
static TaskHandle_t dshot_task_handle = NULL;
static volatile bool streaming = false;
static esp_timer_handle_t dshot_timer = NULL;
static bool dshot_timer_started = false;

static portMUX_TYPE throttle_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint16_t current_throttle = 0;
static void dshot_timer_callback(void *arg);

static void dshot_frame_task(void *arg)
{
    uint16_t erpm;
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!streaming)
            continue;

        uint16_t throttle;
        portENTER_CRITICAL(&throttle_mux);
        throttle = current_throttle;
        portEXIT_CRITICAL(&throttle_mux);

        if (bidir_active)
            dshot_rmt_rx_pause(); // звільнити пін під TX

        dshot_rmt_send(esc_dshot_make_packet(throttle, bidir_active, bidir_active));

        if (bidir_active)
        {
            dshot_rmt_rx_resume_and_arm();
            rx_arm_count++;

            if (rx_arm_count % 2000 == 0)
            {
                uint32_t done, wrong;
                dshot_rmt_get_rx_stats(&done, &wrong);
                ESP_LOGI(TAG, "rx armed x%u, rx_done=%u, wrong_size=%u", rx_arm_count, done, wrong);
            }

            if (dshot_rmt_get_erpm(&erpm))
                ESP_LOGI(TAG, "eRPM: %u", erpm);
        }
    }
}

esp_err_t esc_dshot_init(dshot_mode_t mode, bool is_bidir)
{
    currentMode = mode;
    bidir_active = is_bidir;

    dshot_rmt_set_bitrate(mode == DSHOT_MODE_600 ? 600U : 300U);

    esp_err_t err = dshot_rmt_init(is_bidir);
    if (err != ESP_OK)
        return err;

    if (dshot_task_handle == NULL)
    {
        BaseType_t ok = xTaskCreate(
            dshot_frame_task, "dshot_frame",
            DSHOT_TASK_STACK_SIZE, NULL,
            DSHOT_TASK_PRIORITY, &dshot_task_handle);

        if (ok != pdPASS)
            return ESP_ERR_NO_MEM;
    }

    if (dshot_timer == NULL)
    {
        const esp_timer_create_args_t timer_cfg = {
            .callback = dshot_timer_callback,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "dshot_frame",
            .skip_unhandled_events = true,
        };

        err = esp_timer_create(&timer_cfg, &dshot_timer);
        if (err != ESP_OK)
            return err;
    }

    ESP_LOGI(TAG, "DShot initialized (%s%s)",
             mode == DSHOT_MODE_300 ? "DShot300" : "DShot600",
             is_bidir ? ", bidir" : "");
    return ESP_OK;
}

void esc_dshot_deinit(void)
{
    esc_dshot_stop_stream();

    if (dshot_timer != NULL)
    {
        ESP_ERROR_CHECK(esp_timer_delete(dshot_timer));
        dshot_timer = NULL;
    }

    if (dshot_task_handle)
    {
        vTaskDelete(dshot_task_handle);
        dshot_task_handle = NULL;
    }
    dshot_rmt_deinit();
}

void esc_dshot_start_stream(void)
{
    portENTER_CRITICAL(&throttle_mux);
    current_throttle = 0;
    portEXIT_CRITICAL(&throttle_mux);

    streaming = true;

    if (!dshot_timer_started)
    {
        ESP_ERROR_CHECK(esp_timer_start_periodic(dshot_timer, DSHOT_FRAME_PERIOD_US));
        dshot_timer_started = true;
    }

    ESP_LOGI(TAG, "DShot frame stream started (%u Hz)", DSHOT_FRAME_RATE_HZ);
}

void esc_dshot_stop_stream(void)
{
    streaming = false;

    if (dshot_timer_started)
    {
        ESP_ERROR_CHECK(esp_timer_stop(dshot_timer));
        dshot_timer_started = false;
    }

    portENTER_CRITICAL(&throttle_mux);
    current_throttle = 0;
    portEXIT_CRITICAL(&throttle_mux);

    ESP_LOGI(TAG, "DShot frame stream stopped");
}

void esc_dshot_set_mode(dshot_mode_t mode)
{
    currentMode = mode;
    ESP_LOGI(TAG, "DShot mode changed (%s)",
             mode == DSHOT_MODE_300 ? "DShot300" : "DShot600");
}

void esc_dshot_set_throttle(uint16_t throttle)
{
    if (throttle > 2047)
        throttle = 2047;

    portENTER_CRITICAL(&throttle_mux);
    current_throttle = throttle;
    portEXIT_CRITICAL(&throttle_mux);
}

void esc_dshot_stop(void)
{
    portENTER_CRITICAL(&throttle_mux);
    current_throttle = 0;
    portEXIT_CRITICAL(&throttle_mux);

    ESP_LOGI(TAG, "DShot throttle = 0 (stop)");
}

void esc_dshot_send_command(uint16_t command, bool telemetry)
{
    if (command > 47)
        return;

    uint16_t packet = esc_dshot_make_packet(command, telemetry, bidir_active);
    dshot_rmt_send(packet);
}

uint16_t esc_dshot_make_packet(uint16_t throttle, bool telemetry, bool is_bidir)
{
    if (throttle > 2047)
        throttle = 2047;

    uint16_t data = (throttle << 1) | (telemetry ? 1 : 0);
    uint8_t crc = data ^ (data >> 4) ^ (data >> 8);
    crc &= 0x0F;
    if (is_bidir)
        crc = (~crc) & 0x0F;

    return (data << 4) | crc;
}

static void dshot_timer_callback(void *arg)
{
    xTaskNotifyGive(dshot_task_handle);
}