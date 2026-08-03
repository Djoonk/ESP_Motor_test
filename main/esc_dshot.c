#include "esc_dshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dshot_rmt.h"
#include "esp_timer.h"

static const char *TAG = "ESC_DSHOT";

#define DSHOT_FRAME_PERIOD_US   2000U   // 500 Hz frame rate
#define DSHOT_FRAME_RATE_HZ     500U
#define DSHOT_TASK_STACK_SIZE   4096
#define DSHOT_TASK_PRIORITY     8

// Arming: keep throttle at 0 for DSHOT_ARM_DELAY_MS after the stream starts.
// Driven from the frame task so the BT handler is never blocked.
#define DSHOT_ARM_FRAMES        ((uint32_t)(DSHOT_ARM_DELAY_MS * DSHOT_FRAME_RATE_HZ / 1000))

#define DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE 13
#define EDT_ENABLE_REPEATS    10
#define EDT_ENABLE_DELAY_MS   3

static dshot_mode_t currentMode = DSHOT_MODE_300;
static bool bidir_active = false;
static bool edt_pending = false;
static TaskHandle_t dshot_task_handle = NULL;
static volatile bool streaming = false;
static esp_timer_handle_t dshot_timer = NULL;
static bool dshot_timer_started = false;

static portMUX_TYPE throttle_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint16_t current_throttle = 0;
static volatile uint32_t arming_frames_remaining = 0;
static void dshot_timer_callback(void *arg);

void esc_dshot_enable_edt(void)
{
    edt_pending = true;
}

static void dshot_frame_task(void *arg)
{
    uint32_t erpm;
    uint16_t frame_count = 0;

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!streaming)
            continue;

        // ── EDT enable (once) ──
        if (edt_pending)
        {
            edt_pending = false;
            ESP_LOGI(TAG, "EDT: sending cmd 13 x%d...", EDT_ENABLE_REPEATS);
            for (int i = 0; i < EDT_ENABLE_REPEATS; i++)
            {
                dshot_rmt_send(DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE, false);
                vTaskDelay(pdMS_TO_TICKS(EDT_ENABLE_DELAY_MS));
            }
            ESP_LOGI(TAG, "EDT enabled");
            continue;
        }

        // ── Read throttle (force 0 while the ESC is arming) ──
        uint16_t throttle;
        portENTER_CRITICAL(&throttle_mux);
        if (arming_frames_remaining > 0)
        {
            arming_frames_remaining--;
            throttle = 0;
        }
        else
        {
            throttle = current_throttle;
        }
        portEXIT_CRITICAL(&throttle_mux);

        // ── Bidirectional frame (MichelJansson flow: send + busy-wait reply) ──
        if (bidir_active)
        {
            dshot_rmt_send(throttle, true);

            if (dshot_rmt_wait_erpm(&erpm) == ESP_OK)
            {
                if (frame_count % 100 == 0)
                {
                    ESP_LOGI(TAG, "T:%u eRPM(1LSB=100):%lu", throttle, erpm);
                }
            }
            else if (frame_count % 500 == 0)
            {
                ESP_LOGW(TAG, "No eRPM (T:%u, frames:%u)", throttle, frame_count);
            }

            frame_count++;
        }
        else
        {
            // ── Unidirectional frame ──
            dshot_rmt_send(throttle, false);
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

    ESP_LOGI(TAG, "DShot init: %s, bidir=%d",
             mode == DSHOT_MODE_300 ? "DShot300" : "DShot600", is_bidir);
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
    arming_frames_remaining = DSHOT_ARM_FRAMES;
    portEXIT_CRITICAL(&throttle_mux);

    dshot_rmt_reset_telemetry();
    streaming = true;

    if (!dshot_timer_started)
    {
        ESP_ERROR_CHECK(esp_timer_start_periodic(dshot_timer, DSHOT_FRAME_PERIOD_US));
        dshot_timer_started = true;
    }

    ESP_LOGI(TAG, "Stream started (%u Hz), arming %u ms",
             DSHOT_FRAME_RATE_HZ, DSHOT_ARM_DELAY_MS);
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

    ESP_LOGI(TAG, "Stream stopped");
}

void esc_dshot_set_mode(dshot_mode_t mode)
{
    currentMode = mode;
}

void esc_dshot_set_throttle(uint16_t throttle)
{
    if (throttle > 2047) throttle = 2047;
    portENTER_CRITICAL(&throttle_mux);
    current_throttle = throttle;
    portEXIT_CRITICAL(&throttle_mux);
}

void esc_dshot_stop(void)
{
    portENTER_CRITICAL(&throttle_mux);
    current_throttle = 0;
    portEXIT_CRITICAL(&throttle_mux);
}

void esc_dshot_send_command(uint16_t command, bool telemetry)
{
    if (command > 47) return;
    dshot_rmt_send(command, telemetry);
}

bool esc_dshot_is_bidirectional(void)
{
    return bidir_active;
}

static void dshot_timer_callback(void *arg)
{
    xTaskNotifyGive(dshot_task_handle);
}
