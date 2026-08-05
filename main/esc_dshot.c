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
#define EDT_ENABLE_REPEATS    8
// AM32 dshot.c: a command executes only after 6 CONSECUTIVE identical command
// frames; ANY other frame (throttle 0 included) resets its command_count.
// So cmd 13 must be sent back-to-back with no throttle frames in between.

static dshot_mode_t currentMode = DSHOT_MODE_300;
static bool bidir_active = false;
static TaskHandle_t dshot_task_handle = NULL;
static volatile bool streaming = false;
static esp_timer_handle_t dshot_timer = NULL;
static bool dshot_timer_started = false;

static portMUX_TYPE throttle_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint16_t current_throttle = 0;
static volatile uint32_t arming_frames_remaining = 0;

// Last known EDT values (updated as the ESC cycles through frame types)
static volatile uint32_t telem_erpm = INVALID_TELEMETRY_VALUE; // eRPM/100
static volatile uint32_t telem_voltage = INVALID_TELEMETRY_VALUE; // 0.01 V
static volatile uint32_t telem_current = INVALID_TELEMETRY_VALUE; // 0.01 A
static volatile uint32_t telem_temperature = INVALID_TELEMETRY_VALUE; // degC
static bool edt_ack_logged = false;

// Raw (unscaled) values — phone applies the scale factors
static volatile uint16_t telem_raw_erpm = 0;
static volatile uint8_t  telem_raw_temp = 0;
static volatile uint8_t  telem_raw_voltage = 0;
static volatile uint8_t  telem_raw_current = 0;

// EDT enable state: cmd 13 (with telemetry bit set) is interleaved into the
// normal 500 Hz stream AFTER arming, while the motor is stopped (throttle 0).
// The frame stream stays continuous so the ESC never sees a signal stall.
// See BLHeli_32 Digital_Cmd_Spec.txt: 6x, >= 35 ms apart, motors stopped,
// telemetry bit set in the command frames.
static bool edt_pending = false;
static int edt_cmds_sent = 0;

static void dshot_timer_callback(void *arg);

void esc_dshot_enable_edt(void)
{
    edt_pending = true;
    edt_cmds_sent = 0;
}

static void dshot_frame_task(void *arg)
{
    uint16_t frame_count = 0;

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!streaming)
            continue;

        // ── Read throttle (force 0 while the ESC is arming) ──
        uint16_t throttle;
        bool armed_now = false;
        portENTER_CRITICAL(&throttle_mux);
        if (arming_frames_remaining > 0)
        {
            arming_frames_remaining--;
            throttle = 0;
            if (arming_frames_remaining == 0)
            {
                edt_pending = true;
                armed_now = true;
            }
        }
        else
        {
            throttle = current_throttle;
        }
        portEXIT_CRITICAL(&throttle_mux);

        if (armed_now)
        {
            ESP_LOGI(TAG, "ESC armed, enabling EDT");
        }

        // ── Bidirectional frame (MichelJansson flow: send + busy-wait reply) ──
        if (bidir_active)
        {
            // EDT enable: send cmd 13 (with telemetry bit set) back-to-back,
            // NO throttle frames between them - AM32 requires 6 consecutive
            // identical command frames and resets its counter on any other
            // frame. One frame per timer tick (500 Hz) is consecutive here.
            // Throttle stays 0 until the burst completes, so the motor cannot
            // spool up.
            bool send_cmd = false;
            if (edt_pending)
            {
                send_cmd = true;
                if (++edt_cmds_sent >= EDT_ENABLE_REPEATS)
                {
                    edt_pending = false;
                    ESP_LOGI(TAG, "EDT enable done (cmd 13 x%d)", EDT_ENABLE_REPEATS);
                }
            }

            uint16_t to_send = send_cmd ? DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE : throttle;
            bool sent = dshot_rmt_send(to_send, true);

            dshot_telemetry_t telem;
            if (sent && dshot_rmt_wait_telemetry(&telem) == ESP_OK)
            {
                // Refresh last known EDT value for this frame type
                bool edt_ack = false;
                uint32_t edt_ack_val = 0;
                portENTER_CRITICAL(&throttle_mux);
                switch (telem.type)
                {
                case DSHOT_TELEMETRY_TYPE_eRPM:
                    telem_erpm = telem.value;
                    telem_raw_erpm = telem.raw;
                    break;
                case DSHOT_TELEMETRY_TYPE_TEMPERATURE:
                    telem_temperature = telem.value;
                    telem_raw_temp = (uint8_t)telem.raw;
                    break;
                case DSHOT_TELEMETRY_TYPE_VOLTAGE:
                    telem_voltage = telem.value;
                    telem_raw_voltage = (uint8_t)telem.raw;
                    break;
                case DSHOT_TELEMETRY_TYPE_CURRENT:
                    telem_current = telem.value;
                    telem_raw_current = (uint8_t)telem.raw;
                    break;
                case DSHOT_TELEMETRY_TYPE_STATE_EVENTS:
                    edt_ack = true;
                    edt_ack_val = telem.value;
                    break;
                default:
                    break;
                }
                portEXIT_CRITICAL(&throttle_mux);

                if (edt_ack && !edt_ack_logged)
                {
                    edt_ack_logged = true;
                    ESP_LOGI(TAG, "EDT ack/status frame: 0x%02lx", edt_ack_val);
                }

                if (frame_count % 100 == 0)
                {
                    uint32_t erpm = telem_erpm;
                    uint32_t volt = telem_voltage;
                    uint32_t curr = telem_current;
                    uint32_t temp = telem_temperature;
                    ESP_LOGI(TAG, "T:%u eRPM(LSB=100):%lu  V:%lu.%02lu  A:%lu.%02lu  T:%luC",
                             throttle, erpm,
                             volt / 100, volt % 100,
                             curr / 100, curr % 100,
                             temp);
                }
            }
            else if (arming_frames_remaining == 0 && frame_count % 500 == 0)
            {
                ESP_LOGW(TAG, "No telemetry (T:%u, frames:%u)", throttle, frame_count);
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

    edt_ack_logged = false;
    edt_pending = false;
    edt_cmds_sent = 0;

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
    telem_erpm = INVALID_TELEMETRY_VALUE;
    telem_voltage = INVALID_TELEMETRY_VALUE;
    telem_current = INVALID_TELEMETRY_VALUE;
    telem_temperature = INVALID_TELEMETRY_VALUE;
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

void esc_dshot_get_raw_telemetry(esc_dshot_raw_telemetry_t *out)
{
    portENTER_CRITICAL(&throttle_mux);
    out->erpm        = telem_raw_erpm;
    out->temperature = telem_raw_temp;
    out->voltage     = telem_raw_voltage;
    out->current     = telem_raw_current;
    portEXIT_CRITICAL(&throttle_mux);
}

static void dshot_timer_callback(void *arg)
{
    xTaskNotifyGive(dshot_task_handle);
}
