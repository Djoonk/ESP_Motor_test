#include "dshot_rmt.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esc_pwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "rom/ets_sys.h"
#include "esp_timer.h"

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

// ── DShot timing at 8 MHz (1 tick = 125 ns) ──
// DShot300: bit = 3.33 us = 26.67 ticks
// DShot600: bit = 1.67 us = 13.33 ticks
#define RMT_RESOLUTION_HZ  8000000U

// DShot300 at 8 MHz
#define DSHOT300_T1H_TICKS 20   // 2.50 us
#define DSHOT300_T1L_TICKS 6    // 0.75 us (bit=3.25 us)
#define DSHOT300_T0H_TICKS 10   // 1.25 us
#define DSHOT300_T0L_TICKS 16   // 2.00 us

// DShot600 at 8 MHz
#define DSHOT600_T1H_TICKS 10   // 1.25 us
#define DSHOT600_T1L_TICKS 3    // 0.375 us
#define DSHOT600_T0H_TICKS 5    // 0.625 us
#define DSHOT600_T0L_TICKS 8    // 1.00 us

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
            .resolution_hz = RMT_RESOLUTION_HZ,
            .mem_block_symbols = 64,
            .trans_queue_depth = 1,
            .flags = {
                .invert_out = 1,
                .init_level = 0,
            },
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

static volatile bool rx_enabled = false;

esp_err_t dshot_rmt_send(uint16_t packet)
{
    if (tx_channel == NULL || copy_encoder == NULL)
        return ESP_ERR_INVALID_STATE;

    if (rx_channel != NULL && rx_enabled)
    {
        rmt_disable(rx_channel);
        rx_enabled = false;
    }

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

    err = rmt_tx_wait_all_done(tx_channel, 10);

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
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 128,
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
    if (tx_channel == NULL || copy_encoder == NULL)
        return -1;

    if (rx_enabled)
    {
        rmt_disable(rx_channel);
        rx_enabled = false;
    }

    rmt_receive_config_t rx_cfg = {
        .signal_range_min_ns = 500,
        .signal_range_max_ns = 50000,
    };

    size_t rx_size = rx_buf_size * sizeof(rmt_symbol_word_t);
    esp_err_t err = rmt_receive(rx_channel, rx_buf, rx_size, &rx_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_receive: %s", esp_err_to_name(err));
        return -1;
    }

    rmt_symbol_word_t tx_symbols[16];
    dshot_rmt_encode_packet(packet, tx_symbols);

    const rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };

    err = rmt_transmit(tx_channel, copy_encoder,
                       tx_symbols, sizeof(tx_symbols), &tx_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_transmit: %s", esp_err_to_name(err));
        return -1;
    }

    err = rmt_tx_wait_all_done(tx_channel, 10);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "TX done: %s", esp_err_to_name(err));
        return -1;
    }

    int num_rx = 0;

    if (xSemaphoreTake(rx_done_sem,
                       pdMS_TO_TICKS(timeout_us / 1000 + 1)) == pdTRUE)
    {
        num_rx = (int)rx_num_symbols;
    }
    else
    {
        ESP_LOGW(TAG, "RX timeout (%u us)", timeout_us);
    }

    return num_rx;
}

bool dshot_rmt_decode_gcr(const rmt_symbol_word_t *symbols,
                           int count,
                           uint16_t *decoded_out)
{
    if (count < 21)
        return false;

    int start = 0;
    if (count >= 37)
        start = count - 21;

    // Extract 21 raw GCR bits from RMT symbols
    uint32_t gcr_raw = 0;
    for (int i = start; i < start + 21; i++)
    {
        bool bit_is_one = symbols[i].duration0 > symbols[i].duration1;
        gcr_raw = (gcr_raw << 1) | bit_is_one;
    }

    // Step 2: GCR decode: decoded = encoded ^ (encoded >> 1)
    uint32_t decoded = gcr_raw ^ (gcr_raw >> 1);

    // Step 3: Extract 16-bit data (4 nibbles of 4 bits)
    // The decoded value is 20 bits: [nibble3][nibble2][nibble1][nibble0][crc]
    // We need the upper 16 bits (4 data nibbles)
    uint16_t data = (decoded >> 4) & 0xFFFF;

    // Step 4: Verify CRC — XOR of all 4 nibbles + CRC nibble must be 0
    uint8_t crc_check = 0;
    crc_check ^= (data >> 12) & 0x0F;
    crc_check ^= (data >> 8) & 0x0F;
    crc_check ^= (data >> 4) & 0x0F;
    crc_check ^= data & 0x0F;

    if (crc_check != 0x00)
    {
        // Try inverted CRC (bidirectional DShot uses inverted CRC)
        crc_check = 0;
        uint16_t inverted_data = data;
        uint8_t inverted_crc = (~((decoded >> 4) & 0x0F)) & 0x0F;
        inverted_data = (inverted_data & 0xFFF0) | inverted_crc;

        crc_check = 0;
        crc_check ^= (inverted_data >> 12) & 0x0F;
        crc_check ^= (inverted_data >> 8) & 0x0F;
        crc_check ^= (inverted_data >> 4) & 0x0F;
        crc_check ^= inverted_data & 0x0F;

        if (crc_check != 0x00)
            return false;

        data = inverted_data;
    }

    *decoded_out = data;
    return true;
}
