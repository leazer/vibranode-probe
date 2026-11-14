/**
 * @file test_probe_irq.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "at32f413.h"
#include "at32f413_int.h"
#include "probe_validation.h"
#include "probe_spi.h"
void EXINT0_IRQHandler(void);
void EXINT1_IRQHandler(void);
void EXINT15_10_IRQHandler(void);
static uint32_t pending;
static unsigned acq, diag, usb_calls;
void probe_spi_dma_isr(probe_spi_bus bus) { (void)bus; }
void probe_time_overflow_isr(void) { }
void probe_usb_isr(void) { ++usb_calls; }
void wk_timebase_handler(void) { }
flag_status exint_interrupt_flag_get(uint32_t line) { return pending & line ? SET : RESET; }
void exint_flag_clear(uint32_t line) { pending &= ~line; }
void probe_acquisition_irq(unsigned source) { acq |= 1U << source; }
void probe_validation_irq(unsigned source) { diag |= 1U << source; }
int test_main(void)
{
    pending = EXINT_LINE_0 | EXINT_LINE_1 | EXINT_LINE_10;
    EXINT0_IRQHandler(); EXINT1_IRQHandler(); EXINT15_10_IRQHandler();
    USBFS_L_CAN1_RX0_IRQHandler(); /* CAN1 unused; vector dispatches to USB */
#if PROBE_HARDWARE_VALIDATION == 1
    if (diag != 7 || acq || pending) return __LINE__;
#else
    if (acq != 7 || diag || pending) return __LINE__;
#endif
    if (usb_calls != 1) return __LINE__;
    return 0;
}
