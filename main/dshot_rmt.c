#include "dshot_rmt.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esc_pwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "DSHOT_RMT";

static rmt_channel_handle_t tx_channel = NULL;
static rmt_encoder_handle_t copy_encoder = NULL;
static uint32_t dshot_bitrate_khz = 300;
static rmt_channel_handle_t rx_channel = NULL;
static SemaphoreHandle_t rx_done_sem = NULL;
static volatile size_t rx_num_symbols = 0;

static bool rx_done_callback(rmt_channel_handle_t channel,
                             const rmt_rx_done_event_data_t *edata,
                             void *user_ctx)
{
    rx_num_symbols = edata->num_symbols;
    BaseType_t wake = pdFALSE;
    xSemaphoreGiveFromISR(rx_done_sem, &wake);
    return wake == pdTRUE;
}

#define DSHOT_RMT_TX_TIMEOUT_MS 10

// RMT = 10 MHz, 1 tick = 100 ns
#define DSHOT300_T0H_TICKS 12
#define DSHOT300_T0L_TICKS 21
#define DSHOT300_T1H_TICKS 25
#define DSHOT300_T1L_TICKS 8

// RMT = 10 MHz, 1 tick = 100 ns
// DShot600: bit = 1.67 us = 17 ticks
#define DSHOT600_T0H_TICKS 13  // 0.65 us
#define DSHOT600_T0L_TICKS 20  // 1.00 us
#define DSHOT600_T1H_TICKS 25  // 1.25 us
#define DSHOT600_T1L_TICKS 8   // 0.40 us

void dshot_rmt_set_bitrate(uint32_t bitrate_khz)
{
    if (bitrate_khz == 300U || bitrate_khz == 600U)
    {
        dshot_bitrate_khz = bitrate_khz;
    }
}

esp_err_t dshot_rmt_init(void)
{
    if (tx_channel != NULL)
        dshot_rmt_deinit();

    rmt_tx_channel_config_t config =
        {
            .gpio_num = ESC_PWM_GPIO,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = (dshot_bitrate_khz == 600U)
                     ? 20000000U
                     : 10000000U,
            .mem_block_symbols = 64,
            .trans_queue_depth = 4,
        };

    esp_err_t ret = rmt_new_tx_channel(&config, &tx_channel);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_err_t en = rmt_enable(tx_channel);
    ESP_LOGI(TAG, "rmt_enable result: %s", esp_err_to_name(en));
    if (en != ESP_OK)
        return en;

    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &copy_encoder));

    ESP_LOGI(TAG, "RMT init: tx_channel=%p, encoder=%p",
             tx_channel, copy_encoder);
    return ESP_OK;
}

esp_err_t dshot_rmt_deinit(void)
{
    if (copy_encoder)
    {
        rmt_del_encoder(copy_encoder);
        copy_encoder = NULL;
    }

    if (tx_channel)
    {
        rmt_disable(tx_channel);
        rmt_del_channel(tx_channel);
        tx_channel = NULL;
    }

    ESP_LOGI(TAG, "RMT deinit");
    return ESP_OK;
}

esp_err_t dshot_rmt_send(uint16_t packet)
{
    if (tx_channel == NULL || copy_encoder == NULL)
        return ESP_ERR_INVALID_STATE;

    rmt_symbol_word_t symbols[16];
    dshot_rmt_encode_packet(packet, symbols);

    const rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };

    esp_err_t err = rmt_transmit(tx_channel, copy_encoder,
                                 symbols, sizeof(symbols), &tx_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_transmit: %s", esp_err_to_name(err));
        return err;
    }

    // return rmt_tx_wait_all_done(tx_channel, DSHOT_RMT_TX_TIMEOUT_MS);

    err = rmt_tx_wait_all_done(tx_channel,
                               DSHOT_RMT_TX_TIMEOUT_MS);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "TX done: %s", esp_err_to_name(err));
    }

    return err;
}

void dshot_rmt_encode_packet(uint16_t packet,
                             rmt_symbol_word_t symbols[16])
{
    for (int i = 0; i < 16; i++)
    {
        bool bit = packet & (1 << (15 - i));

        symbols[i].level0 = 1;
        symbols[i].level1 = 0;

        if (dshot_bitrate_khz == 600U)
        {
            if (bit)
            {
                symbols[i].duration0 = DSHOT600_T1H_TICKS;
                symbols[i].duration1 = DSHOT600_T1L_TICKS;
            }
            else
            {
                symbols[i].duration0 = DSHOT600_T0H_TICKS;
                symbols[i].duration1 = DSHOT600_T0L_TICKS;
            }
        }
        else
        {
            if (bit)
            {
                symbols[i].duration0 = DSHOT300_T1H_TICKS;
                symbols[i].duration1 = DSHOT300_T1L_TICKS;
            }
            else
            {
                symbols[i].duration0 = DSHOT300_T0H_TICKS;
                symbols[i].duration1 = DSHOT300_T0L_TICKS;
            }
        }
    }
}

esp_err_t dshot_rmt_init_rx(void)
{
    if (rx_channel != NULL)
        dshot_rmt_deinit_rx();

    if (rx_done_sem == NULL)
        rx_done_sem = xSemaphoreCreateBinary();

    rmt_rx_channel_config_t config = {
        .gpio_num = ESC_PWM_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = (dshot_bitrate_khz == 600U)
                             ? 20000000U
                             : 10000000U,
        .mem_block_symbols = 64,
    };

    esp_err_t ret = rmt_new_rx_channel(&config, &rx_channel);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_new_rx_channel failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rx_done_callback,
    };
    rmt_rx_register_event_callbacks(rx_channel, &cbs, NULL);

    ESP_LOGI(TAG, "RMT RX init: rx_channel=%p", rx_channel);
    return ESP_OK;
}

void dshot_rmt_deinit_rx(void)
{
    if (rx_channel)
    {
        rmt_disable(rx_channel);
        rmt_del_channel(rx_channel);
        rx_channel = NULL;
    }
    ESP_LOGI(TAG, "RMT RX deinit");
}

int dshot_rmt_send_receive(uint16_t packet,
                           rmt_symbol_word_t *rx_buf,
                           size_t rx_buf_size,
                           uint32_t timeout_us)
{
    // ── TX phase (існуючий код) ──
    if (tx_channel == NULL || copy_encoder == NULL)
        return -1;

    rmt_symbol_word_t tx_symbols[16];
    dshot_rmt_encode_packet(packet, tx_symbols);

    const rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };

    esp_err_t err = rmt_transmit(tx_channel, copy_encoder,
                                  tx_symbols, sizeof(tx_symbols),
                                  &tx_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_transmit: %s", esp_err_to_name(err));
        return -1;
    }

    err = rmt_tx_wait_all_done(tx_channel,
                                DSHOT_RMT_TX_TIMEOUT_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "TX done: %s", esp_err_to_name(err));
        return -1;
    }

    // ── TX→RX switch ──
    rmt_disable(tx_channel);
    gpio_set_direction(ESC_PWM_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ESC_PWM_GPIO, GPIO_PULLUP_ONLY);

    // ── RX phase ──
    rmt_enable(rx_channel);

    rmt_receive_config_t rx_cfg = {
        .signal_range_min_ns = 500,
        .signal_range_max_ns = 100000,
    };

    size_t rx_size = rx_buf_size * sizeof(rmt_symbol_word_t);
    err = rmt_receive(rx_channel, rx_buf, rx_size, &rx_cfg);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_receive: %s", esp_err_to_name(err));
        rmt_disable(rx_channel);
        gpio_set_direction(ESC_PWM_GPIO, GPIO_MODE_OUTPUT);
        gpio_set_level(ESC_PWM_GPIO, 0);
        rmt_enable(tx_channel);
        return -1;
    }

    int num_rx = 0;

    if (xSemaphoreTake(rx_done_sem,
                       pdMS_TO_TICKS(timeout_us / 1000 + 1)) == pdTRUE)
    {
        num_rx = (int)rx_num_symbols;
    }

    rmt_disable(rx_channel);

    // ── RX→TX switch ──
    gpio_set_direction(ESC_PWM_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ESC_PWM_GPIO, 0);
    rmt_enable(tx_channel);

    return num_rx;
}

bool dshot_rmt_decode_gcr(const rmt_symbol_word_t *symbols,
                           int count,
                           uint16_t *decoded_out)
{
    if (count < 21)
        return false;

    // Symbol 0 = idle LOW + start bit HIGH (корумпований)
    // Symbols 1-20 = кожен містить LOW попереднього біта + HIGH наступного
    // Біт = (duration1 > duration0) → HIGH тривалість > LOW тривалість
    uint32_t encoded = 0;

    for (int i = 1; i <= 20 && i < count; i++)
    {
        bool bit = symbols[i].duration1 > symbols[i].duration0;

        if (bit)
            encoded |= (1U << (20 - i));
    }

    // GCR decode: running XOR
    uint32_t decoded = encoded;
    decoded ^= decoded >> 1;
    decoded ^= decoded >> 2;
    decoded ^= decoded >> 4;
    decoded ^= decoded >> 8;
    decoded ^= decoded >> 16;
    decoded &= 0xFFFFF;

    // Перевірка CRC: XOR усіх 5 nibble → має бути 0
    uint8_t crc = 0;
    crc ^= (decoded >> 16) & 0x0F;
    crc ^= (decoded >> 12) & 0x0F;
    crc ^= (decoded >> 8) & 0x0F;
    crc ^= (decoded >> 4) & 0x0F;
    crc ^= decoded & 0x0F;

    if (crc != 0)
        return false;

    *decoded_out = (uint16_t)(decoded >> 4);
    return true;
}