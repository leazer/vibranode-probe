/**
 * @file sensor_register.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef SENSOR_REGISTER_H
#define SENSOR_REGISTER_H

#include <stdint.h>

/* One register write. The caller waits after completion, then verifies mask.
 * A zero mask is reserved for self-clearing commands such as reset. */
typedef struct {
    uint8_t reg;
    uint8_t value;
    uint8_t mask;
    uint32_t wait_us;
} sensor_register_setting;

#endif
