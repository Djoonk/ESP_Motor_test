#include "bluetooth_spp.h"
#include "command_handler.h"
#include "esc_controller.h"
#include "esc_protocol.h"
#include "esc_dshot.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_spp_api.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "BT_SPP";

#define SPP_SERVER_NAME "MotorTest_SPP"
#define BT_DEVICE_NAME "MotorTest_ESP32"
#define PACKET_SOF 0xAAU
#define CMD_TELEMETRY 0x10U   // ESP → phone: raw telemetry packet

#define BT_CMD_QUEUE_LEN 8
#define BT_CMD_TASK_STACK 4096
#define BT_CMD_TASK_PRIORITY 5

#define BT_TELEM_TASK_STACK 4096
#define BT_TELEM_TASK_PRIORITY 3
#define BT_TELEM_PERIOD_MS 500

const char *BtRespond = "MotorTest_ESP32_is_connected\r\n";
typedef enum
{
    RX_WAIT_SOF,
    RX_WAIT_COMMAND,
    RX_WAIT_VALUE,
    RX_WAIT_CRC
} protocol_rx_state_t;

typedef struct
{
    uint8_t command;
    uint8_t value;
} bt_command_msg_t;

static protocol_rx_state_t rx_state = RX_WAIT_SOF;
static uint8_t rx_command;
static uint8_t rx_value;

static uint32_t spp_handle = 0;
static bool client_connected = false;
static QueueHandle_t command_queue = NULL;
static TaskHandle_t command_task_handle = NULL;

static void spp_send(const uint8_t *data, size_t length)
{
    if (!client_connected || data == NULL || length == 0)
        return;

    esp_err_t result = esp_spp_write(spp_handle, length, (uint8_t *)data);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "SPP write failed: %s", esp_err_to_name(result));
    }
}

void bluetooth_spp_send(const uint8_t *data, size_t length)
{
    spp_send(data, length);
}

static void bt_command_task(void *arg)
{
    bt_command_msg_t msg;

    while (1)
    {
        if (xQueueReceive(command_queue, &msg, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI(TAG, "Processing cmd=%u value=%u", msg.command, msg.value);
            command_handler_process((ESC_command_t)msg.command, msg.value);
        }
    }
}

static void protocol_rx_byte(uint8_t byte)
{
    // ESP_LOGW(TAG, "Parssing is OK");
    switch (rx_state)
    {
    case RX_WAIT_SOF:
        if (byte == PACKET_SOF)
            rx_state = RX_WAIT_COMMAND;
        break;

    case RX_WAIT_COMMAND:
        rx_command = byte;
        rx_state = RX_WAIT_VALUE;
        break;

    case RX_WAIT_VALUE:
        rx_value = byte;
        rx_state = RX_WAIT_CRC;
        break;

    case RX_WAIT_CRC:
        if (byte == (uint8_t)(PACKET_SOF ^ rx_command ^ rx_value))
        {
            bt_command_msg_t msg = {.command = rx_command, .value = rx_value};

            // Неблокуючий - колбек BT-стека ніколи не чекає.
            if (xQueueSend(command_queue, &msg, 0) != pdTRUE)
                ESP_LOGW(TAG, "Command queue full, dropped cmd=%u", rx_command);
        }
        else
            ESP_LOGW(TAG, "CRC is bad");
        rx_state = RX_WAIT_SOF;
        break;

    default:
        rx_state = RX_WAIT_SOF;
        break;
    }
}

static void spp_callback(esp_spp_cb_event_t event,
                         esp_spp_cb_param_t *param)
{
    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "SPP initialized");
        esp_bt_gap_set_device_name(BT_DEVICE_NAME);
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        break;

    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG, "SPP server started");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        spp_handle = param->srv_open.handle;
        client_connected = true;

        ESP_LOGI(TAG, "Bluetooth client connected");
        bluetooth_spp_send((const uint8_t *)BtRespond, strlen(BtRespond));;

        uint8_t packet[4] = {0xAA, CMD_PROTOCOL, esc_protocol_get(), 0x00};
        packet[3] = packet[0] ^ packet[1] ^ packet[2]; // CRC (XOR)

        bluetooth_spp_send(packet, sizeof(packet));
        
        break;

    case ESP_SPP_CLOSE_EVT:
        ESP_LOGW(TAG, "Bluetooth client disconnected");

        client_connected = false;
        spp_handle = 0;

        // disarm йде через чергу, а не напряму - на випадок,
        // якщо в майбутньому esc_controller_disarm() стане важчою.
        if (command_queue != NULL)
        {
            bt_command_msg_t msg = {.command = CMD_DISARM, .value = 0};
            xQueueSend(command_queue, &msg, 0);
        }

        ESP_LOGW(TAG, "Disarm command queued because Bluetooth was disconnected");
        break;

    case ESP_SPP_DATA_IND_EVT: // прийом та парсінг
        for (uint16_t i = 0; i < param->data_ind.len; ++i)
            protocol_rx_byte(param->data_ind.data[i]);

        break;

    default:
        break;
    }
}

static void bt_telemetry_task(void *arg)
{
    // Wait for BT connection before sending
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(BT_TELEM_PERIOD_MS));

        if (!client_connected)
            continue;

        // Fetch raw (unscaled) telemetry from the ESC
        esc_dshot_raw_telemetry_t raw;
        esc_dshot_get_raw_telemetry(&raw);

        // Packet: [SOF][CMD_TELEMETRY][eRPM_lo][eRPM_hi][temp][voltage][current][CRC]
        // CRC = XOR of all bytes before CRC
        uint8_t pkt[8];
        pkt[0] = 0xAA;
        pkt[1] = CMD_TELEMETRY;
        pkt[2] = (uint8_t)(raw.erpm & 0xFF);        // eRPM low byte
        pkt[3] = (uint8_t)((raw.erpm >> 8) & 0xFF);  // eRPM high byte
        pkt[4] = raw.temperature;
        pkt[5] = raw.voltage;
        pkt[6] = raw.current;

        uint8_t crc = 0;
        for (int i = 0; i < 7; i++)
            crc ^= pkt[i];
        pkt[7] = crc;

        spp_send(pkt, sizeof(pkt));
    }
}

esp_err_t bluetooth_spp_init(void)
{
    // NVS is initialized in app_main() before the controllers, so only guard
    // against the "no free pages" reset here (re-init is harmless if already done).
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
        result == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(result));
        return result;
    }

    command_queue = xQueueCreate(BT_CMD_QUEUE_LEN, sizeof(bt_command_msg_t));
    if (command_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create command queue");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t task_ok = xTaskCreate(
        bt_command_task, "bt_cmd_task",
        BT_CMD_TASK_STACK, NULL,
        BT_CMD_TASK_PRIORITY, &command_task_handle);

    if (task_ok != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create command task");
        return ESP_ERR_NO_MEM;
    }

    task_ok = xTaskCreate(
        bt_telemetry_task, "bt_telem_task",
        BT_TELEM_TASK_STACK, NULL,
        BT_TELEM_TASK_PRIORITY, NULL);

    if (task_ok != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create telemetry task");
        return ESP_ERR_NO_MEM;
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    result = esp_bt_controller_init(&bt_cfg);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(result));
        return result;
    }

    result = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(result));
        return result;
    }

    result = esp_bluedroid_init();
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(result));
        return result;
    }

    result = esp_bluedroid_enable();
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(result));
        return result;
    }

    result = esp_spp_register_callback(spp_callback);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "SPP callback registration failed: %s", esp_err_to_name(result));
        return result;
    }

    result = esp_spp_init(ESP_SPP_MODE_CB);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "SPP init failed: %s", esp_err_to_name(result));
        return result;
    }

    ESP_LOGI(TAG, "Starting Bluetooth Classic SPP");
    return ESP_OK;
}