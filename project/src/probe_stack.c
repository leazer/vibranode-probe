/**
 * @file probe_stack.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <stdint.h>
#include "probe_stack.h"
extern uint32_t __probe_stack_base, __probe_stack_limit;
unsigned probe_stack_capacity(void)
{
    return (unsigned)((uintptr_t)&__probe_stack_limit - (uintptr_t)&__probe_stack_base);
}
unsigned probe_stack_unused(void)
{
    const volatile uint32_t *p = &__probe_stack_base;
    while ((uintptr_t)p < (uintptr_t)&__probe_stack_limit && *p == 0xA5A5A5A5U) ++p;
    return (unsigned)((uintptr_t)p - (uintptr_t)&__probe_stack_base);
}
