#include "esc_measurements.h"
#include "esp_log.h"

static const char *TAG = "measurements";

static adc_oneshot_unit_handle_t s_adc1 = NULL;

esp_err_t esc_measurements_init(void)
{
    if (s_adc1 != NULL) {
        return ESP_OK;
    }

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

    ESP_LOGI(TAG, "ADC initialized: current=GPIO%d, voltage=GPIO%d",
             MEAS_CURRENT_ADC_CH, MEAS_VOLTAGE_ADC_CH);
    return ESP_OK;
}
