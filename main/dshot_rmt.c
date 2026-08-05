#include "dshot_rmt.h"
#include "dshot_rmt_encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "hal/gpio_ll.h"
#include "esc_pwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DSHOT_RMT";

// MichelJansson DShot-ESP32RMT: 40 MHz resolution for ESP32
#define RMT_RESOLUTION_HZ 40000000U

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define MAX_BLOCKS 64
#else
#define MAX_BLOCKS 48
#endif

static rmt_channel_handle_t rmt_rx_channel = NULL;
static rmt_channel_handle_t rmt_tx_channel = NULL;
static rmt_encoder_handle_t dshot_encoder = NULL;

static rmt_receive_config_t rx_config;
static rmt_transmit_config_t tx_config;
static dshot_rmt_throttle_t throttle = {
    .throttle = 0,
    .telemetry_req = false,
};

static bool enabled = false;
static bool mode = false; // true = TX (push-pull), false = RX (open-drain)
static bool is_bidirectional = false;

static uint32_t dshot_bitrate_khz = 300;
static uint16_t telemetry_bit_len_ticks; // one telemetry bit in RMT ticks
static uint32_t telemetry_timeout_us;    // full bidir send+receive window
static rmt_symbol_word_t rx_buf[MAX_BLOCKS];
static uint32_t telemetry_gcr = 0;          // last received GCR frame
static bool telemetry_received = false;

// ── GCR decode: 5 bits -> 4 bits lookup table (0xFF = invalid) ──
static const unsigned char GCR_DECODE_TABLE[32] DRAM_ATTR = {
    0xFF, 0xFF, 0xFF, 0xFF, // 0 - 3
    0xFF, 0xFF, 0xFF, 0xFF, // 4 - 7
    0xFF, 9, 10, 11,        // 8 - 11
    0xFF, 13, 14, 15,       // 12 - 15
    0xFF, 0xFF, 2, 3,       // 16 - 19
    0xFF, 5, 6, 7,          // 20 - 23
    0xFF, 0, 8, 1,          // 24 - 27
    0xFF, 4, 12, 0xFF,      // 28 - 31
};

static unsigned long IRAM_ATTR get_micros(void)
{
    return (unsigned long)(esp_timer_get_time());
}

static esp_err_t IRAM_ATTR wait_for_flag(const bool *flag, const uint32_t timeout_us)
{
    if (!*flag && timeout_us)
    {
        uint32_t m = get_micros();
        uint32_t e = (m + timeout_us);
        if (m > e)
        { // overflow
            while (!*flag && get_micros() > e)
            {
                asm("nop");
            }
        }
        while (!*flag && get_micros() < e)
        {
            asm("nop");
        }
    }

    return *flag ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void IRAM_ATTR disable_rx(void)
{
    ESP_ERROR_CHECK(rmt_disable(rmt_rx_channel));
    // Disable pin open drain to create a clean signal on the wire
    gpio_ll_od_disable(&GPIO, ESC_PWM_GPIO);
}

static void IRAM_ATTR enable_rx(void)
{
    // NOTE: time critical function, execution must not exceed 5-6us

    // Change pin to open drain to allow ESC to drive the wire
    gpio_ll_od_enable(&GPIO, ESC_PWM_GPIO);

    ESP_ERROR_CHECK(rmt_enable(rmt_rx_channel));
}

static void IRAM_ATTR mode_tx(void)
{
    if (!mode)
    {
        disable_rx();
        mode = true;
    }
}

static void IRAM_ATTR mode_rx(void)
{
    if (mode)
    {
        enable_rx();
        mode = false;

        ESP_ERROR_CHECK(rmt_receive(rmt_rx_channel, rx_buf, sizeof(rx_buf), &rx_config));
    }
}

static uint32_t IRAM_ATTR duration_to_bit_len(uint32_t duration, uint32_t len)
{
    return (duration + (len >> 1)) / len;
}

static uint32_t convert_erpm_data_to_erpm_period(uint32_t value); // fwd decl, defined below

static uint32_t IRAM_ATTR push_bits(uint32_t value, uint32_t bit_val, size_t bit_len)
{
    while (bit_len--)
    {
        value <<= 1;
        value |= bit_val;
    }
    return value;
}

/**
 * @param rmt_symbols Pointer to the RMT symbols
 * @param len Number of symbols
 * @return uint32_t raw GCR value
 */
static uint32_t IRAM_ATTR extract_telemetry_gcr(rmt_symbol_word_t *rmt_symbols,
                                                size_t symbol_num,
                                                uint32_t bit_len_ticks)
{
    rmt_symbol_word_t *cur = rmt_symbols;

    // First bit should be a 0 starting bit
    if (cur->level0 != 0)
    {
        return 0;
    }

    int bit_count = 0;
    uint32_t value = 0;
    for (size_t i = 0; i < symbol_num; i++)
    {
        if (!cur->duration0)
            break;
        uint32_t bit_len0 = duration_to_bit_len(cur->duration0, bit_len_ticks);
        if (bit_len0)
        {
            value = push_bits(value, cur->level0, bit_len0);
            bit_count += bit_len0;
        }

        if (!cur->duration1)
            break;
        uint32_t bit_len1 = duration_to_bit_len(cur->duration1, bit_len_ticks);
        if (bit_len1)
        {
            value = push_bits(value, cur->level1, bit_len1);
            bit_count += bit_len1;
        }
        cur++;
    }

    // fill missing bits with 1
    if (bit_count < 21)
    {
        value = push_bits(value, 0x1, 21 - bit_count);
    }

    // First bit is start bit so discard it.
    value &= 0xfffff;

    value = value ^ (value >> 1); // extract GCR

    return value;
}

static bool IRAM_ATTR rx_done_cb(rmt_channel_handle_t ch,
                                 const rmt_rx_done_event_data_t *edata,
                                 void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;

    // parse the received RMT symbols
    telemetry_gcr = extract_telemetry_gcr(edata->received_symbols,
                                          edata->num_symbols,
                                          telemetry_bit_len_ticks);
    // raise flag that telemetry has been received
    telemetry_received = true;

    return high_task_wakeup == pdTRUE;
}

static bool IRAM_ATTR tx_done_cb(rmt_channel_handle_t ch,
                                 const rmt_tx_done_event_data_t *edata,
                                 void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;

    mode_rx();

    return high_task_wakeup == pdTRUE;
}

/**
 * Converts the GCR value into a full EDT telemetry frame, validates its CRC
 * @param value 20-bit GCR value
 * @return dshot_telemetry_t decoded type + scaled value
 */
static dshot_telemetry_t convert_gcr_to_telemetry(uint32_t value)
{
    dshot_telemetry_t out = {
        .type = DSHOT_TELEMETRY_TYPE_INVALID,
        .value = INVALID_TELEMETRY_VALUE,
    };

    if (!value)
    {
        return out;
    }

    // ...shifting 5 bits -> 4 bits (0xff => invalid)
    const unsigned char *table = GCR_DECODE_TABLE;

    uint32_t decoded_value = table[value & 0x1f];
    decoded_value |= (uint32_t)table[(value >> 5) & 0x1f] << 4;
    decoded_value |= (uint32_t)table[(value >> 10) & 0x1f] << 8;
    decoded_value |= (uint32_t)table[(value >> 15) & 0x1f] << 12;

    uint32_t csum = decoded_value;
    csum = csum ^ (csum >> 8); // xor bytes
    csum = csum ^ (csum >> 4); // xor nibbles

    if ((csum & 0xf) != 0xf || decoded_value > 0xffff)
    {
        return out;
    }

    // 12-bit data field: "eeem mmmm mmmm"
    uint16_t data = decoded_value >> 4;

    // Prefix (3-bit exponent + mantissa MSB) tells eRPM from EDT frames
    unsigned type = (data & 0x0f00) >> 8;
    if ((type & 0x01) || (type == 0))
    {
        out.type = DSHOT_TELEMETRY_TYPE_eRPM;
        out.value = convert_erpm_data_to_erpm_period(data);
        out.raw = data;
        return out;
    }

    uint8_t raw = data & 0x00ff;
    switch (type)
    {
    case 0x02: // 0010 -> temperature, 1 LSB = 1 degC
        out.type = DSHOT_TELEMETRY_TYPE_TEMPERATURE;
        out.value = raw;
        out.raw = raw;
        break;
    case 0x04: // 0100 -> voltage; AM32 sends battery_voltage/25 (1 LSB = 0.25 V,
               // scaled to 0.01 V). Multiply by VOLTAGE_SCALE_PPM/1e6 to calibrate
               // against the ESC's real voltage divider (see header).
        out.type = DSHOT_TELEMETRY_TYPE_VOLTAGE;
        out.value = (uint32_t)raw * 25 * VOLTAGE_SCALE_PPM / 1000000ULL;
        out.raw = raw;
        break;
    case 0x06: // 0110 -> current; AM32 v2.20 sends actual_current/50 with
               // actual_current in 0.01 A, so 1 LSB = 0.5 A (scaled to 0.01 A).
        out.type = DSHOT_TELEMETRY_TYPE_CURRENT;
        out.value = (uint32_t)raw * 50;
        out.raw = raw;
        break;
    case 0x08:
        out.type = DSHOT_TELEMETRY_TYPE_DEBUG1;
        out.value = raw;
        break;
    case 0x0a:
        out.type = DSHOT_TELEMETRY_TYPE_DEBUG2;
        out.value = raw;
        break;
    case 0x0c:
        out.type = DSHOT_TELEMETRY_TYPE_DEBUG3;
        out.value = raw;
        break;
    case 0x0e: // 1110 -> state / events
        out.type = DSHOT_TELEMETRY_TYPE_STATE_EVENTS;
        out.value = raw;
        break;
    default:
        break;
    }

    return out;
}

/**
 * Converts eRPM data into an eRPM period
 * @param value 12-bit eRPM data
 * @return uint32_t eRPM (1 LSB = 100 eRPM)
 */
static uint32_t convert_erpm_data_to_erpm_period(uint32_t value)
{
    if (!value || value == INVALID_TELEMETRY_VALUE)
    {
        return INVALID_TELEMETRY_VALUE;
    }

    // eRPM range
    if (value == 0x0fff)
    {
        return 0;
    }

    // Convert value to 16 bit from the GCR telemetry format (eeem mmmm mmmm)
    value = (value & 0x01ff) << ((value & 0xfe00) >> 9);

    if (!value || value == INVALID_TELEMETRY_VALUE)
    {
        return INVALID_TELEMETRY_VALUE;
    }

    // Convert period to erpm * 100
    return (1000000 * 60 / 100 + value / 2) / value;
}

// ── Bitrate / tick pre-calc ──
void dshot_rmt_set_bitrate(uint32_t bitrate_khz)
{
    if (bitrate_khz == 300U || bitrate_khz == 600U)
        dshot_bitrate_khz = bitrate_khz;
}

// ── Init RMT TX + RX channels ──
esp_err_t dshot_rmt_init(bool bidirectional)
{
    if (rmt_tx_channel != NULL)
        dshot_rmt_deinit();

    is_bidirectional = bidirectional;

    uint32_t baudrate = (dshot_bitrate_khz == 600U) ? 600000U : 300000U;
    uint32_t post_delay_us = 3;

    if (is_bidirectional)
    {
        uint32_t telem_baudrate = baudrate * 5 / 4;
        telemetry_bit_len_ticks = (unsigned int)((float)RMT_RESOLUTION_HZ / telem_baudrate);

        uint32_t throttle_frame_length = ((float)1000000 / baudrate * 16);
        uint32_t telemetry_frame_length = ((float)1000000 / telem_baudrate * 21);
        telemetry_timeout_us = throttle_frame_length + 30 + telemetry_frame_length + 40; // throttle frame + receive delay + telemetry frame + RMT delay
    }

    ESP_LOGI(TAG, "Install DShot RMT encoder");
    dshot_rmt_encoder_config_t encoder_config = {
        .resolution = RMT_RESOLUTION_HZ,
        .baud_rate = baudrate,
        .bidirectional = is_bidirectional,
        .post_delay_us = post_delay_us, // extra delay between each frame
    };

    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder));

    if (is_bidirectional)
    {
        ESP_LOGI(TAG, "Create RMT RX channel");
        const rmt_rx_channel_config_t rx_channel_config = {
            .gpio_num = ESC_PWM_GPIO,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = RMT_RESOLUTION_HZ,
            .mem_block_symbols = MAX_BLOCKS,
            .flags = {
                .io_loop_back = true,
            },
            .intr_priority = 1,
        };
        ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_config, &rmt_rx_channel));

        ESP_LOGI(TAG, "Register RX callback");
        rmt_rx_event_callbacks_t rx_callbacks = {
            .on_recv_done = rx_done_cb,
        };
        ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rmt_rx_channel, &rx_callbacks, NULL));

        rx_config = (rmt_receive_config_t){
            .signal_range_min_ns = 150,  // dshot 1200 shortest pulse is 0.313us
            .signal_range_max_ns = 25000 // dshot 150 longest pulse is 5.0us
        };
    }

    ESP_LOGI(TAG, "Create RMT TX channel");
    const rmt_tx_channel_config_t tx_channel_config = {
        .gpio_num = ESC_PWM_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT, // a clock that can provide needed resolution
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = MAX_BLOCKS,
        .trans_queue_depth = 1, // set the number of transactions that can be pending in the background
        .intr_priority = 1,
        .flags = {
            .invert_out = is_bidirectional,
            .io_loop_back = true,
        }};
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_config, &rmt_tx_channel));

    if (is_bidirectional)
    {
        ESP_LOGI(TAG, "Register TX callback");
        rmt_tx_event_callbacks_t tx_callbacks = {
            .on_trans_done = tx_done_cb,
        };
        ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(rmt_tx_channel, &tx_callbacks, NULL));
    }

    tx_config = (rmt_transmit_config_t){
        .loop_count = 0,
        .flags = {
            .queue_nonblocking = true,
        },
    };

    throttle.throttle = 0;
    throttle.telemetry_req = false;

    // Enable TX channel (fast, non-blocking). The ESC arming sequence
    // (3.2 s of throttle=0 frames) is driven from the frame task so the
    // BT command handler is never blocked and the link stays alive.
    ESP_ERROR_CHECK(rmt_enable(rmt_tx_channel));
    mode = true;
    enabled = true;

    ESP_LOGI(TAG, "RMT init: %s, bidir=%d",
             dshot_bitrate_khz == 600U ? "DShot600" : "DShot300",
             bidirectional);
    return ESP_OK;
}

esp_err_t dshot_rmt_deinit(void)
{
    if (dshot_encoder)
    {
        rmt_del_encoder(dshot_encoder);
        dshot_encoder = NULL;
    }

    if (rmt_rx_channel)
    {
        if (enabled && !mode) // RX channel is only enabled while in RX state
        {
            ESP_ERROR_CHECK(rmt_disable(rmt_rx_channel));
        }
        ESP_ERROR_CHECK(rmt_del_channel(rmt_rx_channel));
        rmt_rx_channel = NULL;
    }

    if (rmt_tx_channel)
    {
        if (enabled)
        {
            ESP_ERROR_CHECK(rmt_disable(rmt_tx_channel));
        }
        ESP_ERROR_CHECK(rmt_del_channel(rmt_tx_channel));
        rmt_tx_channel = NULL;
    }

    enabled = false;
    mode = false;
    is_bidirectional = false;
    telemetry_received = false;
    telemetry_gcr = 0;
    ESP_LOGI(TAG, "RMT deinit");
    return ESP_OK;
}

bool dshot_rmt_send(uint16_t value, bool telemetry_req)
{
    if (!enabled)
        return false;

    throttle.throttle = value;
    throttle.telemetry_req = telemetry_req;
    telemetry_received = false;

    mode_tx();

    // Non-blocking transmit. With trans_queue_depth==1 this can return
    // ESP_ERR_INVALID_STATE if the previous TX isn't complete yet (a rare,
    // harmless race) - caller decides whether to treat it as a dropped frame.
    return rmt_transmit(rmt_tx_channel, dshot_encoder, &throttle, sizeof(throttle), &tx_config) == ESP_OK;
}

esp_err_t dshot_rmt_wait_telemetry(dshot_telemetry_t *out)
{
    out->type = DSHOT_TELEMETRY_TYPE_INVALID;
    out->value = INVALID_TELEMETRY_VALUE;

    if (!enabled || !is_bidirectional)
        return ESP_ERR_INVALID_STATE;

    if (wait_for_flag(&telemetry_received, telemetry_timeout_us) == ESP_OK)
    {
        *out = convert_gcr_to_telemetry(telemetry_gcr);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t dshot_rmt_wait_erpm(uint32_t *erpm)
{
    *erpm = INVALID_TELEMETRY_VALUE;

    dshot_telemetry_t t;
    if (dshot_rmt_wait_telemetry(&t) == ESP_OK && t.type == DSHOT_TELEMETRY_TYPE_eRPM)
    {
        *erpm = t.value;
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

void dshot_rmt_get_telemetry(dshot_telemetry_t *out)
{
    out->type = DSHOT_TELEMETRY_TYPE_INVALID;
    out->value = INVALID_TELEMETRY_VALUE;

    if (!enabled || !is_bidirectional)
        return;

    *out = convert_gcr_to_telemetry(telemetry_gcr);
}

uint32_t dshot_rmt_get_erpm(void)
{
    dshot_telemetry_t t;
    dshot_rmt_get_telemetry(&t);
    return (t.type == DSHOT_TELEMETRY_TYPE_eRPM) ? t.value : INVALID_TELEMETRY_VALUE;
}

void dshot_rmt_reset_telemetry(void)
{
    telemetry_received = false;
    telemetry_gcr = 0;
}

uint32_t dshot_rmt_get_raw_gcr(void)
{
    return telemetry_gcr;
}