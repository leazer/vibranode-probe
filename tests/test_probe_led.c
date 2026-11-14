/**
 * @file test_probe_led.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "at32f413_wk_config.h"
#include "probe_led.h"

/* Recorded pin levels: 1 = driven active (LED lit). */
static struct {
    uint64_t now;
    int r, g, b;
} hw;

uint64_t probe_time_us(void) { return hw.now; }

void gpio_bits_set(gpio_type *p, uint16_t pins)
{
    (void)p;
    if (pins & LED_R_PIN) hw.r = 0;
    if (pins & LED_G_PIN) hw.g = 0;
    if (pins & LED_B_PIN) hw.b = 0;
}

void gpio_bits_reset(gpio_type *p, uint16_t pins)
{
    (void)p;
    if (pins & LED_R_PIN) hw.r = 1;
    if (pins & LED_G_PIN) hw.g = 1;
    if (pins & LED_B_PIN) hw.b = 1;
}

#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
#define W (hw.r && hw.g && hw.b) /* white = all channels lit */

int test_main(void)
{
    /* Init lands on state 0 (blue steady); the first poll corrects it to 6
     * while no host has enumerated. */
    probe_led_init();
    CHECK(hw.r == 0 && hw.g == 0 && hw.b == 1);

    /* State 0: USB up, no session — blue steady, never blinks. */
    hw.now += 5000000;
    probe_led_update(0);
    CHECK(hw.r == 0 && hw.g == 0 && hw.b == 1);

    /* State 6: no USB host — white 1 Hz 500/500, starts lit. */
    probe_led_update(6);
    CHECK(W);
    hw.now += 499000;
    probe_led_update(6);
    CHECK(W);                      /* still inside the lit half */
    hw.now += 2000;
    probe_led_update(6);
    CHECK(!hw.r && !hw.g && !hw.b); /* dark half */
    hw.now += 500000;
    probe_led_update(6);
    CHECK(W);

    /* State 1: standby — blue 2 s heartbeat 100/1900, starts lit. */
    probe_led_update(1);
    CHECK(hw.r == 0 && hw.g == 0 && hw.b == 1);
    hw.now += 99000;
    probe_led_update(1);
    CHECK(hw.b == 1);
    hw.now += 2000;
    probe_led_update(1);
    CHECK(!hw.b);                  /* 1900 ms dark half begins */
    hw.now += 1898000;
    probe_led_update(1);
    CHECK(!hw.b);
    hw.now += 3000;
    probe_led_update(1);
    CHECK(hw.b == 1);              /* 2 s period done */

    /* State 2: measuring — green double pulse 100/50/100/750. */
    probe_led_update(2);
    CHECK(hw.r == 0 && hw.g == 1 && hw.b == 0);
    hw.now += 99000;
    probe_led_update(2);
    CHECK(hw.g == 1);
    hw.now += 2000;
    probe_led_update(2);
    CHECK(!hw.g);                  /* 50 ms gap */
    hw.now += 47000;
    probe_led_update(2);
    CHECK(!hw.g);
    hw.now += 3000;
    probe_led_update(2);
    CHECK(hw.g == 1);              /* second pulse */
    hw.now += 97000;
    probe_led_update(2);
    CHECK(hw.g == 1);
    hw.now += 3000;
    probe_led_update(2);
    CHECK(!hw.g);                  /* 750 ms tail */
    hw.now += 747000;
    probe_led_update(2);
    CHECK(!hw.g);
    hw.now += 3000;
    probe_led_update(2);
    CHECK(hw.g == 1);              /* period restarts lit */

    /* State 3: calibrating — purple 5 Hz 100/100. */
    probe_led_update(3);
    CHECK(hw.r == 1 && hw.g == 0 && hw.b == 1);
    hw.now += 100000;
    probe_led_update(3);
    CHECK(!hw.r && !hw.b);
    hw.now += 100000;
    probe_led_update(3);
    CHECK(hw.r == 1 && hw.b == 1);

    /* State 4: measuring warning — red 5 Hz 100/100. */
    probe_led_update(4);
    CHECK(hw.r == 1 && hw.g == 0 && hw.b == 0);
    hw.now += 100000;
    probe_led_update(4);
    CHECK(!hw.r);
    hw.now += 100000;
    probe_led_update(4);
    CHECK(hw.r == 1);

    /* State 5: measuring fault — red steady. */
    probe_led_update(5);
    CHECK(hw.r == 1 && hw.g == 0 && hw.b == 0);
    hw.now += 5000000;
    probe_led_update(5);
    CHECK(hw.r == 1);

    /* A state switch restarts the new pattern from its first lit phase even
     * when the old one was mid-dark. */
    probe_led_update(6);
    hw.now += 500000; /* dark half */
    probe_led_update(6);
    CHECK(!hw.r && !hw.g && !hw.b);
    probe_led_update(1);
    CHECK(hw.b == 1);

    /* Out-of-range states are ignored entirely. */
    probe_led_update(7);
    CHECK(hw.b == 1);
    return 0;
}
