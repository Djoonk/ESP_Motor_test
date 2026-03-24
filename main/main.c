#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <stdbool.h>

/* 
    піни, на яки можна підключити кнопку PIO 16, 17, 18, 19, 21, 22, 23. 
    - button is GPIO19

*/
#define BUTTON_PIN 19
#define LED_PIN 2
// #define BLINK_GPIO CONFIG_BLINK_GPIO
static const char *TAG = "test";


// ============== function ==============//
static uint8_t s_led_state = 0;
static void configure_led(void)
{  
    ESP_LOGI(TAG, "LED IS CONFIGURED");
    gpio_reset_pin(2);
    gpio_set_direction(2, GPIO_MODE_OUTPUT);
}
static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(2, s_led_state);
}

void init_button() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,           // Працюємо на вхід
        .pull_up_en = GPIO_PULLUP_ENABLE,  // ВМИКАЄМО внутрішню підтяжку
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE      // Поки що без переривань
    };
    gpio_config(&io_conf);
}
bool led_state = false;
void toggle_led() {
    led_state = !led_state; 
    gpio_set_level(LED_PIN, led_state);
}

// ============== function ==============//


void app_main(void) {
    // 1. Налаштування LED (Вихід)
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // 2. Налаштування Кнопки (Вхід + PullUp)
    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY); // Пін підтягнутий до 3.3V

    int mode = 0;
    int last_state = 1;
    int cnt = 0;
    
    while(1) {
        int current_state = gpio_get_level(BUTTON_PIN);

        // Детекція натискання (Перехід з 1 в 0)
        if (last_state == 1 && current_state == 0) {
            mode = (mode + 1) % 3; // Перемикаємо 0 -> 1 -> 2 -> 0
            printf("Зміна режиму! Поточний: %d\n", mode);
            vTaskDelay(pdMS_TO_TICKS(10)); // Антибрязок (Debounce)
        }
        last_state = current_state;

        // Візуалізація режимів
        if (mode == 0) {
            gpio_set_level(LED_PIN, 0);
        } else if (mode == 1) {
            s_led_state = !s_led_state;
            blink_led(); // 
            vTaskDelay(pdMS_TO_TICKS(500)); 
        } else if (mode == 2) {
            s_led_state = !s_led_state;
            blink_led(); // 
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Даємо дихати RTOS
    }
}
