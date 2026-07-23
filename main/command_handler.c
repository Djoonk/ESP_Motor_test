#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "command_handler.h"
#include "esc_protocol.h"
#include "esc_controller.h"

static void command_select_protocol(esc_protocol_t protocol)
{
    esc_controller_disarm();
    esc_protocol_select(protocol);
}

void command_handler_process(const char *command,
                             char *response,
                             size_t response_size)
{
    if (command == NULL || response == NULL || response_size == 0)
    {
        return;
    }

    if (strcmp(command, "protocol pwm") == 0)
    {
        command_select_protocol(ESC_PROTOCOL_PWM);

        snprintf(response, response_size,
                 "OK PROTOCOL PWM");
        return;
    }
    
    if (strcmp(command, "protocol dshot300") == 0)
    {
        command_select_protocol(ESC_PROTOCOL_DSHOT300);

        snprintf(response, response_size,
                 "OK PROTOCOL DSHOT300");

        return;
    }

    if (strcmp(command, "protocol dshot600") == 0)
    {
        command_select_protocol(ESC_PROTOCOL_DSHOT600);

        snprintf(response, response_size,
                 "OK PROTOCOL DSHOT600");
        return;
    }

    if (strcmp(command, "protocol bdshot") == 0)
    {
        command_select_protocol(ESC_PROTOCOL_BIDIRECTIONAL_DSHOT);

        snprintf(response, response_size,
                 "OK PROTOCOL BDSHOT");
        return;
    }

    if (strcmp(command, "arm") == 0)
    {
        esc_controller_arm();
        snprintf(response, response_size, "OK ARM\r\n");
        return;
    }

    if (strcmp(command, "disarm") == 0)
    {
        esc_controller_disarm();
        snprintf(response, response_size, "OK DISARM\r\n");
        return;
    }

    if (strcmp(command, "stop") == 0)
    {
        esc_controller_stop();
        snprintf(response, response_size, "OK STOP\r\n");
        return;
    }

    if (strncmp(command, "throttle ", 9) == 0)
    {
        const char *value_text = command + 9;
        char *end_ptr = NULL;

        long value = strtol(value_text, &end_ptr, 10);

        if (*value_text == '\0' || *end_ptr != '\0')
        {
            snprintf(response, response_size,
                     "ERROR THROTTLE_FORMAT\r\n");
            return;
        }

        if (value < 0 || value > 100)
        {
            snprintf(response, response_size,
                     "ERROR THROTTLE_RANGE\r\n");
            return;
        }

        stand_state_t state = esc_controller_get_state();

        if (state != STAND_ARMED && state != STAND_RUNNING)
        {
            snprintf(response, response_size,
                     "ERROR NOT_ARMED\r\n");
            return;
        }

        esc_controller_set_throttle((uint16_t)value);

        snprintf(response, response_size,
                 "OK THROTTLE %ld\r\n", value);
        return;
    }

    if (strcmp(command, "estop") == 0)
    {
        esc_controller_emergency_stop();
        snprintf(response, response_size, "OK ESTOP\r\n");
        return;
    }

    if (strcmp(command, "reset_estop") == 0)
    {
        esc_controller_reset_emergency_stop();
        snprintf(response, response_size, "OK RESET_ESTOP\r\n");
        return;
    }

    if (strcmp(command, "status") == 0)
    {
        stand_state_t state = esc_controller_get_state();

        switch (state)
        {
        case STAND_DISARMED:
            snprintf(response, response_size, "STATUS DISARMED\r\n");
            break;

        case STAND_ARMED:
            snprintf(response, response_size, "STATUS ARMED\r\n");
            break;

        case STAND_RUNNING:
            snprintf(response, response_size, "STATUS RUNNING\r\n");
            break;

        case STAND_EMERGENCY_STOP:
            snprintf(response, response_size, "STATUS EMERGENCY_STOP\r\n");
            break;

        default:
            snprintf(response, response_size, "STATUS UNKNOWN\r\n");
            break;
        }

        return;
    }

    snprintf(response, response_size, "ERROR UNKNOWN_COMMAND\r\n");
}
