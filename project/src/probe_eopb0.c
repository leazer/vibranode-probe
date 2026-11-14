/**
 * @file probe_eopb0.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-10
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "at32f413_wk_config.h"
#include "at32f413_flash.h"

/* EOPB0 (USD offset 0x10) selects the SRAM split: 0xA5 = 32KB general RAM +
 * 32KB flash-access SRAM (factory default), 0xFA = full 64KB general RAM.
 * The firmware is linked for 64KB, so __main's scatter load would zero ZI
 * into nonexistent memory before main() runs; Reset_Handler parks MSP in the
 * guaranteed lower 32KB and calls this guard before SystemInit. It returns
 * only when EOPB0 already selects 64KB.
 *
 * Programming follows the Artery AN0026 flow: flash_unlock() MUST come
 * before the USD operations or the usdulks handshake never completes (first
 * hardware attempt hung exactly there). After a verified program the guard
 * blinks purple ~2 s and resets so the option byte reloads; if the part
 * still comes up in 32KB mode, one power cycle applies it.
 *
 * LED timeline (active-low, same wiring as probe_led):
 *   4 purple blinks   guard engaged, about to program
 *   (dark ~50 ms)     USD erase + program in progress
 *   purple ~4 Hz 2 s  programmed and verified, resetting now
 *   red ~8 Hz forever program verify failed
 *   nothing           hang inside the flash library — report it */
#define EOPB0_64K_VALUE 0xFA

static void early_delay(unsigned ms)
{
    while (ms--) {
        volatile uint32_t loops = 1600; /* ≈1 ms at the 8 MHz reset HICK */
        while (loops--) ;
    }
}

static void early_led(unsigned purple, unsigned red)
{
    /* Active-low: drive low to light. */
    if (purple) { gpio_bits_reset(LED_R_GPIO_PORT, LED_R_PIN);
                  gpio_bits_reset(LED_B_GPIO_PORT, LED_B_PIN); }
    else        { gpio_bits_set(LED_R_GPIO_PORT, LED_R_PIN);
                  gpio_bits_set(LED_B_GPIO_PORT, LED_B_PIN); }
    if (red) gpio_bits_reset(LED_R_GPIO_PORT, LED_R_PIN);
    else gpio_bits_set(LED_R_GPIO_PORT, LED_R_PIN);
}

void probe_eopb0_early_config(void)
{
    unsigned i;
    if ((USD->eopb0 & 0xFF) == EOPB0_64K_VALUE) return;
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    {
        gpio_init_type gpio;
        gpio_default_para_init(&gpio);
        gpio.gpio_pins = LED_R_PIN | LED_G_PIN | LED_B_PIN;
        gpio.gpio_mode = GPIO_MODE_OUTPUT;
        gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio.gpio_pull = GPIO_PULL_NONE;
        gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init(LED_R_GPIO_PORT, &gpio);
    }
    for (i = 0; i < 4; ++i) { /* "guard engaged" marker */
        early_led(1, 0); early_delay(125);
        early_led(0, 0); early_delay(125);
    }
    flash_unlock();
    flash_user_system_data_erase();
    flash_user_system_data_program((uint32_t)&USD->eopb0, EOPB0_64K_VALUE);
    if ((USD->eopb0 & 0xFF) == EOPB0_64K_VALUE) {
        for (i = 0; i < 8; ++i) { /* confirmed: ~2 s, then reload via reset */
            early_led(1, 0); early_delay(125);
            early_led(0, 0); early_delay(125);
        }
        nvic_system_reset();
    }
    while (1) { /* verify failed: fast red */
        early_led(0, 1); early_delay(60);
        early_led(0, 0); early_delay(60);
    }
}
