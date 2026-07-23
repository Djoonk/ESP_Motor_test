#pragma once

#include <stddef.h>

/*
 * Обробляє одну текстову команду.
 *
 * command       - наприклад: "arm" або "status"
 * response      - буфер, куди буде записана відповідь
 * response_size - розмір буфера response
 */
void command_handler_process(const char *command,
                             char *response,
                             size_t response_size);