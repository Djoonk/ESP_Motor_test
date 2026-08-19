#pragma once

#include <stddef.h>

typedef enum {
    CMD_ARM             = 0x01,
    CMD_DISARM          = 0x02,
    CMD_STOP            = 0x03,
    CMD_THROTTLE        = 0x04,
    CMD_PROTOCOL        = 0x05,
    CMD_ESTOP           = 0x06,
    CMD_RESET_ESTOP     = 0x07,
    CMD_SET_MOTOR_POLES = 0x08,
    CMD_TARE            = 0x09
}ESC_command_t;


void command_handler_process(ESC_command_t command, uint8_t value);