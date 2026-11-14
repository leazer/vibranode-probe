/**
 * @file probe_time.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-06
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_TIME_H
#define PROBE_TIME_H
#include <stdint.h>
/* Initialize once before sensor interrupts. Reset only while acquisition is stopped.
 * Callers must never mask interrupts for a complete 65.536 ms timer period. */
void probe_time_init(void);
uint64_t probe_time_us(void);
void probe_time_overflow_isr(void);
#endif
