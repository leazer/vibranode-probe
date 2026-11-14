/**
 * @file probe_stack.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_STACK_H
#define PROBE_STACK_H
/* Paint unused stack first in main, after the C runtime and before hardware init.
 * init is boot-only: repainting during operation would erase high-water history.
 * Read-only, main-loop scan.
 * Watermark is measured evidence, not a proof of the worst possible nesting. */
unsigned probe_stack_unused(void);
unsigned probe_stack_capacity(void);
void probe_stack_init(void);
#endif
