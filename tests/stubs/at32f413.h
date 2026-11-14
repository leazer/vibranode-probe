/**
 * @file at32f413.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-06
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_TEST_AT32_H
#define PROBE_TEST_AT32_H
#include <stdint.h>
#define TMR11 11
#define CRM_TMR11_PERIPH_CLOCK 11
#define TMR_OVF_FLAG 1
#define TMR_OVF_INT 1
#define TMR1_TRG_HALL_TMR11_IRQn 26
#define EXINT0_IRQn 16
#define EXINT1_IRQn 17
#define EXINT10_IRQn 40
#define EXINT15_10_IRQn 40
void exint_flag_clear(uint32_t flags);
#define TRUE 1
#define FALSE 0
#define SET 1
#define EXINT_LINE_0 0x00000001U
#define EXINT_LINE_1 0x00000002U
#define EXINT_LINE_10 0x00000400U
extern uint32_t fake_counter, fake_pending, fake_primask, fake_wrap_on_read;
uint32_t tmr_counter_value_get(int timer);
static __inline uint32_t __get_PRIMASK(void) { return fake_primask; }
#define __disable_irq() ((void)(fake_primask = 1))
static __inline void __set_PRIMASK(uint32_t value) { fake_primask = value; }
static __inline void crm_periph_clock_enable(int timer, int state) { (void)timer; (void)state; }
static __inline void tmr_counter_enable(int timer, int state) { (void)timer; (void)state; }
static __inline void tmr_reset(int timer) { (void)timer; fake_counter = 0; fake_pending = 0; }
static __inline void tmr_base_init(int timer, uint32_t period, uint32_t div) { (void)timer; (void)period; (void)div; fake_pending = 1; }
static __inline void tmr_counter_value_set(int timer, uint32_t value) { (void)timer; fake_counter = value; }
static __inline int tmr_flag_get(int timer, int flag) { (void)timer; (void)flag; return fake_pending; }
static __inline void tmr_flag_clear(int timer, int flag) { (void)timer; (void)flag; fake_pending = 0; }
static __inline void tmr_interrupt_enable(int timer, int flag, int state) { (void)timer; (void)flag; (void)state; }
static __inline void NVIC_ClearPendingIRQ(int irq) { (void)irq; }
static __inline void nvic_irq_enable(int irq, int preempt, int sub) { (void)irq; (void)preempt; (void)sub; }
#endif
