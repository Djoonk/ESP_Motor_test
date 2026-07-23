#include "dshot_rmt.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esc_pwm.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "DSHOT_RMT";

static rmt_channel_handle_t tx_channel = NULL;
static rmt_encoder_handle_t copy_encoder = NULL;
static uint32_t dshot_bitrate_khz = 300;

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