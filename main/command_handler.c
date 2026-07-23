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


void command_handler_process(ESC_command_t command, uint8_t value)
{
    switch (command)
    {
    case CMD_ARM:
        esc_controller_arm();
        break;

    case CMD_DISARM:
        esc_controller_disarm();
        break;

    case CMD_STOP:
        esc_controller_stop();
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

    default:
        break;
    }
}