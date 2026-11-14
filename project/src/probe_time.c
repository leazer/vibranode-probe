/**
 * @file probe_time.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-06
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "probe_time.h"
#include "at32f413.h"

typedef struct {
    uint64_t elapsed_us;
} probe_time_context;

/* Accessed only with interrupts masked, including the overflow handler. */
static probe_time_context time_context;

void probe_time_init(void)
{
    uint32_t saved_mask = __get_PRIMASK();
    __disable_irq();
    crm_periph_clock_enable(CRM_TMR11_PERIPH_CLOCK, TRUE);
    tmr_reset(TMR11);
    /* Approved clock tree: APB2=100 MHz, timer clock=2*APB2=200 MHz. */
    tmr_base_init(TMR11, 65535, 199);
    tmr_counter_value_set(TMR11, 0);
    tmr_flag_clear(TMR11, TMR_OVF_FLAG); /* base_init generates an update */
    time_context.elapsed_us = 0;
    NVIC_ClearPendingIRQ(TMR1_TRG_HALL_TMR11_IRQn);
    tmr_interrupt_enable(TMR11, TMR_OVF_INT, TRUE);
    nvic_irq_enable(TMR1_TRG_HALL_TMR11_IRQn, 2, 0);
    tmr_counter_enable(TMR11, TRUE);
    __set_PRIMASK(saved_mask);
}

uint64_t probe_time_us(void)
{
    uint32_t saved_mask = __get_PRIMASK();
    uint32_t counter;
    uint64_t result;
    __disable_irq();
    result = time_context.elapsed_us;
    counter = tmr_counter_value_get(TMR11);
    if (tmr_flag_get(TMR11, TMR_OVF_FLAG) == SET) {
        /* Includes a wrap between the first counter read and the flag read. */
        result += 65536ULL;
        counter = tmr_counter_value_get(TMR11);
    }
    result += counter;
    __set_PRIMASK(saved_mask);
    return result;
}

void probe_time_overflow_isr(void)
{
    uint32_t saved_mask = __get_PRIMASK();
    __disable_irq();
    if (tmr_flag_get(TMR11, TMR_OVF_FLAG) == SET) {
        time_context.elapsed_us += 65536ULL;
        tmr_flag_clear(TMR11, TMR_OVF_FLAG);
    }
    __set_PRIMASK(saved_mask);
}
