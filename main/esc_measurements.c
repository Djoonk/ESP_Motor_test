#include "esc_measurements.h"
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

static const char *TAG = "measurements";

static adc_oneshot_unit_handle_t s_adc1 = NULL;
static adc_cali_handle_t s_adc_cali = NULL;

esp_err_t esc_measurements_init(void)
{
    if (s_adc1 != NULL)
        return ESP_OK;

    adc_oneshot_unit_init_cfg_t init_cfg = 
    {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_adc1);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = 
    {
        .atten = MEAS_ADC_ATTEN,
        .bitwidth = MEAS_ADC_WIDTH_BITS,
    };

    // Current channel: ACHS-7125 on GPIO34 (ADC_CHANNEL_6)
    err = adc_oneshot_config_channel(s_adc1, MEAS_CURRENT_ADC_CH, &chan_cfg);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "config current ch failed: %s", esp_err_to_name(err));
        return err;
    }

    // Voltage channel: divider on GPIO35 (ADC_CHANNEL_7)
    err = adc_oneshot_config_channel(s_adc1, MEAS_VOLTAGE_ADC_CH, &chan_cfg);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "config voltage ch failed: %s", esp_err_to_name(err));
        return err;
    }

    // Calibration: line fitting uses factory eFuse values to correct
    // the ADC gain/Vref non-linearity.
    adc_cali_line_fitting_config_t cali_cfg = 
    {
        .unit_id = ADC_UNIT_1,
        .atten = MEAS_ADC_ATTEN,
        .bitwidth = MEAS_ADC_WIDTH_BITS,
    };
    err = adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc_cali);
    if (err != ESP_OK) 
    {
        ESP_LOGW(TAG, "ADC calibration unavailable, using linear map: %s",
                 esp_err_to_name(err));
        s_adc_cali = NULL;
    }

    ESP_LOGI(TAG, "ADC initialized: current=GPIO%d, voltage=GPIO%d, cali=%s",
             MEAS_CURRENT_ADC_CH, MEAS_VOLTAGE_ADC_CH,
             s_adc_cali != NULL ? "on" : "off");
    return ESP_OK;
}

// ============================================================================
// ADC Voltage read 
// ============================================================================

esp_err_t esc_measurements_read_voltage(float *volts)
{
    if (s_adc1 == NULL || volts == NULL) 
    {
        return ESP_ERR_INVALID_STATE;
    }

    int raw = 0;
    int64_t acc = 0;
    esp_err_t err = ESP_OK;

    for (uint32_t i = 0; i < MEAS_SAMPLE_COUNT; i++) 
    {
        err = adc_oneshot_read(s_adc1, MEAS_VOLTAGE_ADC_CH, &raw);
        if (err != ESP_OK)
            return err;
        acc += raw;
    }

    float avg_raw = (float)acc / (float)MEAS_SAMPLE_COUNT;
    float pin_mv;

    if (s_adc_cali != NULL) 
    {
        int cali_mv = 0;
        err = adc_cali_raw_to_voltage(s_adc_cali, (int)avg_raw, &cali_mv);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "cali raw_to_voltage failed: %s", esp_err_to_name(err));
            return err;
        }
        pin_mv = (float)cali_mv;
    } else {
        pin_mv = avg_raw * (3300.0f / 4095.0f);
    }

    // Voltage at divider node -> battery voltage
    *volts = pin_mv / 1000.0f / MEAS_DIVIDER_K - MEAS_VOLTAGE_OFFSET_V; // коєфіцієнт
    return ESP_OK;
}

// ============================================================================
// ACHS-7125 driver
// ============================================================================

esp_err_t esc_measurements_read_current(float *amps)
{
    if (s_adc1 == NULL || amps == NULL) 
        return ESP_ERR_INVALID_STATE;

    int raw = 0;
    int64_t acc = 0;
    esp_err_t err = ESP_OK;

    for (uint32_t i = 0; i < MEAS_CURRENT_SAMPLES; i++) 
    {
        err = adc_oneshot_read(s_adc1, MEAS_CURRENT_ADC_CH, &raw);
        if (err != ESP_OK)
            return err;
        acc += raw;
    }

    float avg_raw = (float)acc / (float)MEAS_CURRENT_SAMPLES;
    float pin_mv;

    if (s_adc_cali != NULL) 
    {
        int cali_mv = 0;
        err = adc_cali_raw_to_voltage(s_adc_cali, (int)avg_raw, &cali_mv);
        if (err != ESP_OK) 
        {
            ESP_LOGE(TAG, "cali raw_to_voltage failed: %s", esp_err_to_name(err));
            return err;
        }
        pin_mv = (float)cali_mv;
    } else {
        pin_mv = avg_raw * (3300.0f / 4095.0f);
    }

    // ACHS-7125: V_out = Vzero + sensitivity * I.
    // Thrust-test current is unidirectional: clamp negative to 0.
    float i = (pin_mv - MEAS_CURRENT_ADC_OFFSET_MV - MEAS_VZERO_MV) / MEAS_SENS_MV_PER_A;
    if (i < 0.0f)
        i = 0.0f;

    // EMA low-pass between telemetry reads to further smooth ADC noise.
    static float s_filtered = 0.0f;
    static bool s_first = true;
    if (s_first) {
        s_filtered = i;
        s_first = false;
    } else {
        s_filtered = MEAS_EMA_ALPHA * i + (1.0f - MEAS_EMA_ALPHA) * s_filtered;
    }
    *amps = s_filtered;
    return ESP_OK;
}

// ============================================================================
// HX711 (load cell) bit-bang driver
// ============================================================================

// static int32_t s_hx711_tare = 0;
// static bool s_hx711_ready = false;

// esp_err_t hx711_init(void)
// {
//     if (s_hx711_ready)
//         return ESP_OK;

//     gpio_config_t sck_cfg = {
//         .pin_bit_mask = (1ULL << HX711_SCK_GPIO),
//         .mode = GPIO_MODE_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE,
//     };
//     esp_err_t err = gpio_config(&sck_cfg);
//     if (err != ESP_OK) 
//     {
//         ESP_LOGE(TAG, "HX711 SCK config failed: %s", esp_err_to_name(err));
//         return err;
//     }
//     gpio_set_level(HX711_SCK_GPIO, 0);

//     gpio_config_t dout_cfg = 
//     {
//         .pin_bit_mask = (1ULL << HX711_DOUT_GPIO),
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE,
//     };
//     err = gpio_config(&dout_cfg);
//     if (err != ESP_OK) 
//     {
//         ESP_LOGE(TAG, "HX711 DOUT config failed: %s", esp_err_to_name(err));
//         return err;
//     }

//     s_hx711_ready = true;
//     ESP_LOGI(TAG, "HX711 initialized: SCK=GPIO%d, DOUT=GPIO%d",
//              HX711_SCK_GPIO, HX711_DOUT_GPIO);
//     return ESP_OK;
// }

// esp_err_t hx711_read_raw(int32_t *raw)
// {
//     if (!s_hx711_ready)
//         return ESP_ERR_INVALID_STATE;

//     // Wait until DOUT goes LOW = data ready.
//     uint32_t timeout = HX711_TIMEOUT_US;
//     while (gpio_get_level(HX711_DOUT_GPIO) != 0) 
//     {
//         if (--timeout == 0)
//             return ESP_ERR_TIMEOUT;
//         esp_rom_delay_us(1);
//     }

//     int32_t value = 0;
//     for (int i = 0; i < 24; i++) 
//     {
//         gpio_set_level(HX711_SCK_GPIO, 1);
//         value <<= 1;
//         if (gpio_get_level(HX711_DOUT_GPIO))
//             value |= 1;
//         gpio_set_level(HX711_SCK_GPIO, 0);
//     }

//     // 25th pulse: channel A, gain 128 (also starts next conversion).
//     gpio_set_level(HX711_SCK_GPIO, 1);
//     gpio_set_level(HX711_SCK_GPIO, 0);

//     // Sign-extend 24-bit two's complement.
//     if (value & 0x800000)
//         value |= ~0xFFFFFF;

//     *raw = value;
//     return ESP_OK;
// }

// esp_err_t hx711_read_raw_avg(int32_t *avg)
// {
//     // if (avg == NULL)
//     //     return ESP_ERR_INVALID_STATE;

//     const int count = 16;
//     int64_t sum = 0;
//     int ok = 0;
//     for (int i = 0; i < count; i++) 
//     {
//         int32_t v;
//         if (hx711_read_raw(&v) == ESP_OK) 
//         {
//             sum += v;
//             ok++;
//         }
//         esp_rom_delay_us(50); // settle between samples
//     }
//     if (ok == 0)
//         return ESP_ERR_TIMEOUT;

//     *avg = (int32_t)(sum / ok);
//     return ESP_OK;
// }

// esp_err_t hx711_tare(void)
// {
//     int32_t avg = 0;
//     esp_err_t err = hx711_read_raw_avg(&avg);
//     if (err != ESP_OK)
//         return err;

//     s_hx711_tare = avg;
//     ESP_LOGI(TAG, "HX711 tare set: %ld", (long)s_hx711_tare);
//     return ESP_OK;
// }

// esp_err_t hx711_read_grams(float *grams)
// {
//     if (grams == NULL)
//         return ESP_ERR_INVALID_STATE;

//     int32_t avg = 0;
//     esp_err_t err = hx711_read_raw_avg(&avg);
//     if (err != ESP_OK)
//         return err;

//     if (HX711_SCALE_COUNTS_PER_G <= 0.0f) 
//     {
//         // Not calibrated yet: show raw counts so the user can compute
//         // HX711_SCALE_COUNTS_PER_G = (raw_with_load - raw_tare) / weight_g.
//         ESP_LOGI(TAG, "HX711 raw_avg=%ld tare=%ld (uncalibrated)",
//                  (long)avg, (long)s_hx711_tare);
//         *grams = 0.0f;
//         return ESP_OK;
//     }

//     *grams = (float)(avg - s_hx711_tare) / HX711_SCALE_COUNTS_PER_G;
//     ESP_LOGI(TAG, "HX711 thrust=%.1f g (raw_avg=%ld)", *grams, (long)avg);
//     return ESP_OK;
// }
