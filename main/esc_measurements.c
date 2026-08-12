#include "esc_measurements.h"
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "measurements";

static adc_oneshot_unit_handle_t s_adc1 = NULL;
static adc_cali_handle_t s_adc_cali = NULL;

esp_err_t esc_measurements_init(void)
{
    if (s_adc1 != NULL)
        return ESP_OK;

    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_adc1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = MEAS_ADC_ATTEN,
        .bitwidth = MEAS_ADC_WIDTH_BITS,
    };

    // Current channel: ACHS-7125 on GPIO34 (ADC_CHANNEL_6)
    err = adc_oneshot_config_channel(s_adc1, MEAS_CURRENT_ADC_CH, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "config current ch failed: %s", esp_err_to_name(err));
        return err;
    }

    // Voltage channel: divider on GPIO35 (ADC_CHANNEL_7)
    err = adc_oneshot_config_channel(s_adc1, MEAS_VOLTAGE_ADC_CH, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "config voltage ch failed: %s", esp_err_to_name(err));
        return err;
    }

    // Calibration: line fitting uses factory eFuse values to correct
    // the ADC gain/Vref non-linearity.
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = MEAS_ADC_ATTEN,
        .bitwidth = MEAS_ADC_WIDTH_BITS,
    };
    err = adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc_cali);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration unavailable, using linear map: %s",
                 esp_err_to_name(err));
        s_adc_cali = NULL;
    }

    ESP_LOGI(TAG, "ADC initialized: current=GPIO%d, voltage=GPIO%d, cali=%s",
             MEAS_CURRENT_ADC_CH, MEAS_VOLTAGE_ADC_CH,
             s_adc_cali != NULL ? "on" : "off");
    return ESP_OK;
}

esp_err_t esc_measurements_read_voltage(float *volts)
{
    if (s_adc1 == NULL || volts == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw = 0;
    int64_t acc = 0;
    esp_err_t err = ESP_OK;

    for (uint32_t i = 0; i < MEAS_SAMPLE_COUNT; i++) {
        err = adc_oneshot_read(s_adc1, MEAS_VOLTAGE_ADC_CH, &raw);
        if (err != ESP_OK)
            return err;
        acc += raw;
    }

    float avg_raw = (float)acc / (float)MEAS_SAMPLE_COUNT;
    float pin_mv;

    if (s_adc_cali != NULL) {
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

esp_err_t esc_measurements_read_current(float *amps)
{
    if (s_adc1 == NULL || amps == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw = 0;
    int64_t acc = 0;
    esp_err_t err = ESP_OK;

    for (uint32_t i = 0; i < MEAS_CURRENT_SAMPLES; i++) {
        err = adc_oneshot_read(s_adc1, MEAS_CURRENT_ADC_CH, &raw);
        if (err != ESP_OK)
            return err;
        acc += raw;
    }

    float avg_raw = (float)acc / (float)MEAS_CURRENT_SAMPLES;
    float pin_mv;

    if (s_adc_cali != NULL) {
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
