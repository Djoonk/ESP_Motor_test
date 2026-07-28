#include "dshot_rmt.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esc_pwm.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "DSHOT_RMT";

#define DSHOT_RMT_TX_TIMEOUT_MS 20
#define DSHOT_RX_MIN_NS 800
#define DSHOT_RX_MAX_NS 8000
#define DSHOT_ERPM_GCR_BITS 21

// RMT = 10 MHz, 1 tick = 100 ns
#define DSHOT300_T0H_TICKS 12
#define DSHOT300_T0L_TICKS 21
#define DSHOT300_T1H_TICKS 25
#define DSHOT300_T1L_TICKS 8

// RMT = 20 MHz, 1 tick = 50 ns
#define DSHOT600_T0H_TICKS 13
#define DSHOT600_T0L_TICKS 20
#define DSHOT600_T1H_TICKS 25
#define DSHOT600_T1L_TICKS 8

static rmt_channel_handle_t tx_channel = NULL;
static rmt_encoder_handle_t copy_encoder = NULL;
static uint32_t dshot_bitrate_khz = 300;

static rmt_channel_handle_t rx_channel = NULL;
static rmt_symbol_word_t rx_symbols[DSHOT_ERPM_GCR_BITS];
static bool bidir_enabled = false;

static portMUX_TYPE erpm_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint16_t last_erpm = 0;
static volatile bool erpm_ready = false;

static volatile uint32_t rx_done_count = 0;
static volatile uint32_t rx_done_wrong_size = 0;

// GCR: value ^ (value >> 1). CRC інвертований — так ESC відповідає в bidir-режимі.
static uint16_t decode_gcr_frame(const rmt_symbol_word_t *symbols)
{
    uint32_t gcr = 0;
    for (int i = 0; i < DSHOT_ERPM_GCR_BITS; i++)
        gcr = (gcr << 1) | (symbols[i].duration0 > symbols[i].duration1);

    uint32_t decoded = gcr ^ (gcr >> 1);
    uint16_t data = (decoded & 0xFFFF) >> 4;
    uint16_t crc_recv = decoded & 0x0F;
    uint16_t crc_calc = (~(data ^ (data >> 4) ^ (data >> 8))) & 0x0F;

    if (crc_recv != crc_calc)
        return 0;
    return data & 0x07FF;
}

// IRAM_ATTR: викликається з ISR RMT-драйвера, має бути коротким, без логів.
static bool IRAM_ATTR rx_done_cb(rmt_channel_handle_t ch,
                                 const rmt_rx_done_event_data_t *edata,
                                 void *ctx)
{
    rx_done_count++;
    if (edata->num_symbols != DSHOT_ERPM_GCR_BITS)
        rx_done_wrong_size++;
    if (edata->num_symbols == DSHOT_ERPM_GCR_BITS)
    {
        uint16_t erpm = decode_gcr_frame(edata->received_symbols);
        if (erpm != 0)
        {
            portENTER_CRITICAL_ISR(&erpm_mux);
            last_erpm = erpm;
            erpm_ready = true;
            portEXIT_CRITICAL_ISR(&erpm_mux);
        }
    }
    return false;
}

static esp_err_t rx_channel_create(void)
{
    rmt_rx_channel_config_t cfg = {
        .gpio_num = ESC_PWM_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000U,
        .mem_block_symbols = 64,
    };

    esp_err_t err = rmt_new_rx_channel(&cfg, &rx_channel);
    if (err != ESP_OK)
        return err;

    rmt_rx_event_callbacks_t cbs = {.on_recv_done = rx_done_cb};
    err = rmt_rx_register_event_callbacks(rx_channel, &cbs, NULL);
    if (err != ESP_OK)
        return err;

    return rmt_enable(rx_channel);
}

static void rx_channel_destroy(void)
{
    if (!rx_channel)
        return;
    rmt_disable(rx_channel);
    rmt_del_channel(rx_channel);
    rx_channel = NULL;
}

esp_err_t dshot_rmt_rx_pause(void)
{
    if (!rx_channel)
        return ESP_ERR_INVALID_STATE;
    return rmt_disable(rx_channel);
}

esp_err_t dshot_rmt_rx_resume_and_arm(void)
{
    if (!rx_channel)
        return ESP_ERR_INVALID_STATE;

    esp_err_t err = rmt_enable(rx_channel);
    if (err != ESP_OK)
        return err;

    rmt_receive_config_t rx_cfg = {
        .signal_range_min_ns = DSHOT_RX_MIN_NS,
        .signal_range_max_ns = DSHOT_RX_MAX_NS,
    };
    return rmt_receive(rx_channel, rx_symbols, sizeof(rx_symbols), &rx_cfg);
}

bool dshot_rmt_get_erpm(uint16_t *erpm_out)
{
    bool ready;
    portENTER_CRITICAL(&erpm_mux);
    ready = erpm_ready;
    if (ready)
    {
        *erpm_out = last_erpm;
        erpm_ready = false;
    }
    portEXIT_CRITICAL(&erpm_mux);
    return ready;
}

void dshot_rmt_set_bitrate(uint32_t bitrate_khz)
{
    if (bitrate_khz == 300U || bitrate_khz == 600U)
        dshot_bitrate_khz = bitrate_khz;
}

esp_err_t dshot_rmt_init(bool is_bidir)
{
    if (tx_channel != NULL)
        dshot_rmt_deinit();

    bidir_enabled = is_bidir;

    rmt_tx_channel_config_t config = {
        .gpio_num = ESC_PWM_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = (dshot_bitrate_khz == 600U) ? 20000000U : 10000000U,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .flags.invert_out = is_bidir,
        .flags.io_loop_back = is_bidir,
        .flags.io_od_mode = is_bidir,
    };

    esp_err_t ret = rmt_new_tx_channel(&config, &tx_channel);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = rmt_enable(tx_channel);
    if (ret != ESP_OK)
        return ret;

    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &copy_encoder));

    if (is_bidir)
    {
        ret = rx_channel_create();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "rx_channel_create failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    ESP_LOGI(TAG, "RMT init: tx=%p rx=%p bidir=%d", tx_channel, rx_channel, is_bidir);
    return ESP_OK;
}

esp_err_t dshot_rmt_deinit(void)
{
    rx_channel_destroy();

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

    const rmt_transmit_config_t tx_cfg = {.loop_count = 0};

    esp_err_t err = rmt_transmit(tx_channel, copy_encoder, symbols, sizeof(symbols), &tx_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "rmt_transmit: %s", esp_err_to_name(err));
        return err;
    }

    err = rmt_tx_wait_all_done(tx_channel, DSHOT_RMT_TX_TIMEOUT_MS);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "TX done: %s", esp_err_to_name(err));

    return err;
}

void dshot_rmt_encode_packet(uint16_t packet, rmt_symbol_word_t symbols[16])
{
    for (int i = 0; i < 16; i++)
    {
        bool bit = packet & (1 << (15 - i));
        symbols[i].level0 = 1;
        symbols[i].level1 = 0;

        if (dshot_bitrate_khz == 600U)
        {
            symbols[i].duration0 = bit ? DSHOT600_T1H_TICKS : DSHOT600_T0H_TICKS;
            symbols[i].duration1 = bit ? DSHOT600_T1L_TICKS : DSHOT600_T0L_TICKS;
        }
        else
        {
            symbols[i].duration0 = bit ? DSHOT300_T1H_TICKS : DSHOT300_T0H_TICKS;
            symbols[i].duration1 = bit ? DSHOT300_T1L_TICKS : DSHOT300_T0L_TICKS;
        }
    }
}

void dshot_rmt_get_rx_stats(uint32_t *done_count, uint32_t *wrong_size_count)
{
    *done_count = rx_done_count;
    *wrong_size_count = rx_done_wrong_size;
}