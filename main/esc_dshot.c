#include "esc_dshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dshot_rmt.h"
#include "esp_timer.h"

static const char *TAG = "ESC_DSHOT";

#define DSHOT_FRAME_PERIOD_US 500U
#define DSHOT_FRAME_RATE_HZ 2000U
#define DSHOT_TASK_STACK_SIZE 4096
#define DSHOT_TASK_PRIORITY 2
#define DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE 13

static dshot_mode_t currentMode = DSHOT_MODE_300;
static TaskHandle_t dshot_task_handle = NULL;
static esp_timer_handle_t dshot_timer = NULL;
static volatile bool streaming = false;
static bool dshot_timer_started = false;
static volatile uint16_t current_throttle = 0;

static portMUX_TYPE throttle_mux = portMUX_INITIALIZER_UNLOCKED;
static void dshot_timer_callback(void *arg);

static volatile bool bidirectional_mode = false;
static volatile uint8_t pole_count = 14;
static volatile bool edt_enable_pending = false;

static portMUX_TYPE telemetry_mux = portMUX_INITIALIZER_UNLOCKED;
static dshot_telemetry_t last_telemetry = {0};

static rmt_symbol_word_t bidir_rx_buf[128];

static void parse_edt_frame(uint16_t decoded, dshot_telemetry_t *tel);

static void dshot_frame_task(void *arg)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!streaming)
            continue;

        if (edt_enable_pending)
        {
            edt_enable_pending = false;
            ESP_LOGI(TAG, "EDT: sending cmd 13 x6...");
            for (int i = 0; i < 6; i++)
            {
                dshot_rmt_send(
                    esc_dshot_make_packet(
                        DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE, false));
                vTaskDelay(pdMS_TO_TICKS(3));
            }
            ESP_LOGI(TAG, "EDT enabled");
            continue;
        }

        uint16_t throttle;
        portENTER_CRITICAL(&throttle_mux);
        throttle = current_throttle;
        portEXIT_CRITICAL(&throttle_mux);

        bool bidir;
        portENTER_CRITICAL(&throttle_mux);
        bidir = bidirectional_mode;
        portEXIT_CRITICAL(&throttle_mux);

        if (bidir)
        {
            uint16_t packet = esc_dshot_make_packet(throttle, true);
            int rx_count = dshot_rmt_send_receive(
                packet, bidir_rx_buf,
                sizeof(bidir_rx_buf) / sizeof(rmt_symbol_word_t),
                500);

            dshot_telemetry_t tel = {0};

            if (rx_count <= 0)
            {
                ESP_LOGW(TAG, "bidir rx_count=%d (no response)", rx_count);
            }
            else
            {
                uint16_t decoded;
                if (dshot_rmt_decode_gcr(bidir_rx_buf,
                                         rx_count, &decoded))
                {
                    tel.raw_value = decoded;
                    tel.valid = true;
                    parse_edt_frame(decoded, &tel);
                }
                else
                {
                    ESP_LOGW(TAG, "bidir GCR decode fail, rx_count=%d "
                             "sym0=[%d,%d] sym1=[%d,%d] sym2=[%d,%d]",
                             rx_count,
                             bidir_rx_buf[0].duration0,
                             bidir_rx_buf[0].duration1,
                             bidir_rx_buf[1].duration0,
                             bidir_rx_buf[1].duration1,
                             bidir_rx_buf[2].duration0,
                             bidir_rx_buf[2].duration1);
                }
            }

            portENTER_CRITICAL(&telemetry_mux);
            last_telemetry = tel;
            portEXIT_CRITICAL(&telemetry_mux);

            ESP_LOGI(TAG, "T:%u eRPM:%u RPM:%u V:%.1f I:%.1f T:%u°C %s",
                     throttle, tel.erpm, tel.rpm,
                     tel.voltage, tel.current, tel.temperature,
                     tel.valid ? "OK" : "FAIL");
        }
        else
        {
            dshot_rmt_send(
                esc_dshot_make_packet(throttle, false));
        }
    }
}

esp_err_t esc_dshot_init(dshot_mode_t mode)
{
    currentMode = mode;

    dshot_rmt_set_bitrate(
        mode == DSHOT_MODE_600 ? 600U : 300U);

    esp_err_t err = dshot_rmt_init();
    if (err != ESP_OK)
        return err;

    esp_err_t err_rx = dshot_rmt_init_rx();
    if (err_rx != ESP_OK)
        ESP_LOGW(TAG, "RX init failed: %s",
                 esp_err_to_name(err_rx));

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
        {
            return err;
        }
    }

    ESP_LOGI(TAG, "DShot initialized (%s)",
             mode == DSHOT_MODE_300 ? "DShot300" : "DShot600");
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
    dshot_rmt_deinit_rx();
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
        ESP_ERROR_CHECK(
            esp_timer_start_periodic(dshot_timer,
                                     DSHOT_FRAME_PERIOD_US));

        dshot_timer_started = true;
    }

    ESP_LOGI(TAG, "DShot frame stream started (%u Hz)",
             DSHOT_FRAME_RATE_HZ);
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

    uint16_t packet = esc_dshot_make_packet(command, telemetry);
    dshot_rmt_send(packet);
}

static uint8_t dshot_crc(uint16_t value)
{
    uint8_t crc = 0;

    crc ^= value;
    crc ^= value >> 4;
    crc ^= value >> 8;

    crc = crc & 0x0F;

    // Bidirectional DShot uses inverted CRC
    if (bidirectional_mode)
        crc = (~crc) & 0x0F;

    return crc;
}

uint16_t esc_dshot_make_packet(uint16_t throttle, bool telemetry)
{
    if (throttle > 2047)
        throttle = 2047;

    uint16_t packet = (throttle << 1);

    if (telemetry)
        packet |= 1;

    packet = (packet << 4) | dshot_crc(packet);

    return packet;
}

static void dshot_timer_callback(void *arg)
{
    xTaskNotifyGive(dshot_task_handle);
}

void esc_dshot_enable_edt(void)
{
    ESP_LOGI(TAG, "EDT enable pending");
    edt_enable_pending = true;
}

static void parse_edt_frame(uint16_t decoded, dshot_telemetry_t *tel)
{
    uint8_t type = (decoded >> 13) & 0x07;
    uint8_t value = (decoded >> 4) & 0xFF;

    switch (type)
    {
    case 0x00: // eRPM low range
        tel->erpm = value;
        tel->rpm = tel->erpm * 2 / pole_count;
        break;
    case 0x01: // Temperature (°C)
        tel->temperature = value;
        break;
    case 0x02: // Voltage (0.25V per step)
        tel->voltage = value * 0.25f;
        break;
    case 0x03: // Current (1A per step)
        tel->current = (float)value;
        break;
    case 0x04: // eRPM [512..1022]
        tel->erpm = 512 + (value * 2);
        tel->rpm = tel->erpm * 2 / pole_count;
        break;
    case 0x05: // eRPM [1024..2044]
        tel->erpm = 1024 + (value * 4);
        tel->rpm = tel->erpm * 2 / pole_count;
        break;
    case 0x06: // eRPM [2048..4088]
        tel->erpm = 2048 + (value * 8);
        tel->rpm = tel->erpm * 2 / pole_count;
        break;
    case 0x07: // eRPM [4096..8176]
        tel->erpm = 4096 + (value * 16);
        tel->rpm = tel->erpm * 2 / pole_count;
        break;
    }
}

void esc_dshot_set_bidirectional(bool enable)
{
    if (enable && !bidirectional_mode)
    {
        esc_dshot_enable_edt();
        ESP_LOGI(TAG, "Bidirectional DShot enabled");
    }
    else if (!enable && bidirectional_mode)
    {
        ESP_LOGI(TAG, "Bidirectional DShot disabled");
    }

    portENTER_CRITICAL(&throttle_mux);
    bidirectional_mode = enable;
    portEXIT_CRITICAL(&throttle_mux);

    portENTER_CRITICAL(&telemetry_mux);
    last_telemetry = (dshot_telemetry_t){0};
    portEXIT_CRITICAL(&telemetry_mux);
}

bool esc_dshot_is_bidirectional(void)
{
    return bidirectional_mode;
}

void esc_dshot_set_pole_count(uint8_t poles)
{
    if (poles > 0)
        pole_count = poles;
}

dshot_telemetry_t esc_dshot_get_telemetry(void)
{
    dshot_telemetry_t tel;
    portENTER_CRITICAL(&telemetry_mux);
    tel = last_telemetry;
    portEXIT_CRITICAL(&telemetry_mux);
    return tel;
}
