#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "esp_log.h"

#include "command_handler.h"
#include "esc_protocol.h"
#include "esc_controller.h"
#include "esc_dshot.h"
#include "esc_measurements.h"
#include "HX711.h"

static const char *TAG = "ESP_COMMAND";

static void command_select_protocol(esc_protocol_t protocol)
{
    esc_controller_disarm();
    esc_protocol_select(protocol);
}

void command_handler_process(ESC_command_t command, uint8_t value)
{
    switch (command)
    {
    case CMD_ARM:
        esc_controller_arm();
        ESP_LOGW(TAG, "ESC is armed");
        break;

    case CMD_DISARM:
        esc_controller_disarm();
        ESP_LOGW(TAG, "ESC is disarmed");
        break;

    case CMD_STOP:
        esc_controller_stop();
        ESP_LOGW(TAG, "ESC is stoped");
        break;

    case CMD_THROTTLE:
        if (value <= 100U) {
            const stand_state_t state = esc_controller_get_state();

            if (state == STAND_ARMED || state == STAND_RUNNING) {
                esc_controller_set_throttle(value);
            }
        }
        break;

    case CMD_PROTOCOL:
        if (value <= ESC_PROTOCOL_BIDIRECTIONAL_DSHOT) {
            command_select_protocol((esc_protocol_t)value);
        }
        break;

    case CMD_ESTOP:
        esc_controller_emergency_stop();
        break;

    case CMD_RESET_ESTOP:
        esc_controller_reset_emergency_stop();
        break;

    case CMD_SET_MOTOR_POLES:
        // value = number of pole pairs (poles/2), e.g. 14-pole motor -> 7.
        // This converts reported eRPM to mechanical shaft RPM in telemetry.
        esc_dshot_set_motor_pole_pairs(value);
        break;

    case CMD_SET_ZERO:
        if (HX711_zero() == ESP_OK)
            ESP_LOGI(TAG, "HX711 tare set");
        else
            ESP_LOGE(TAG, "HX711 tare failed");
        break;

    default:
        break;
    }
}