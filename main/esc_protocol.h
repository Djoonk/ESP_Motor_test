#ifndef ESC_PROTOCOL_H
#define ESC_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    ESC_PROTOCOL_PWM = 0,
    ESC_PROTOCOL_DSHOT300,
    ESC_PROTOCOL_DSHOT600,
    ESC_PROTOCOL_BIDIRECTIONAL_DSHOT
} esc_protocol_t;

void esc_protocol_init(void);
void esc_protocol_select(esc_protocol_t protocol);
esc_protocol_t esc_protocol_get(void);
void esc_protocol_set_throttle(uint16_t throttle);
void esc_protocol_stop(void);
bool esc_protocol_is_dshot(void);

#endif