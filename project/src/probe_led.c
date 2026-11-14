/**
 * @file probe_led.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "at32f413_wk_config.h"
#include "probe_time.h"
#include "probe_led.h"

typedef struct {
    unsigned state;
    unsigned step;       /* index into the state's phase list */
    uint64_t flip_us;
} probe_led_context;

static probe_led_context led;

/* Channel masks: bit0 = R, bit1 = G, bit2 = B. Each state is a cyclic list
 * of (mask, phase length in us); a single 0-us phase is a steady state.
 * Vocabulary (2026-10-10 redesign, tuned on the board 2026-10-11):
 *   0 USB up, no session  blue   steady
 *   1 standby             blue   2 s heartbeat, 100/1900 ms
 *   2 measuring           green  1 Hz double pulse 100/50/100/750 ms
 *   3 calibrating         purple 5 Hz, 100/100 ms
 *   4 measuring warning   red    5 Hz, 100/100 ms
 *   5 measuring fault     red    steady
 *   6 no USB host         white  1 Hz, 500/500 ms (only reachable from the
 *                        production poll; never appears on the wire) */
static const struct {
    uint8_t mask;
    uint32_t us;
} steps[7][4] = {
    {{0x4, 0}},
    {{0x4, 100000}, {0x0, 1900000}},
    {{0x2, 100000}, {0x0, 50000}, {0x2, 100000}, {0x0, 750000}},
    {{0x5, 100000}, {0x0, 100000}},
    {{0x1, 100000}, {0x0, 100000}},
    {{0x1, 0}},
    {{0x7, 500000}, {0x0, 500000}},
};
static const uint8_t step_count[7] = {1, 2, 4, 2, 2, 1, 2};

static void apply(unsigned state, unsigned step)
{
    unsigned mask = steps[state][step].mask;
    if (mask & 0x1) gpio_bits_reset(LED_R_GPIO_PORT, LED_R_PIN);
    else gpio_bits_set(LED_R_GPIO_PORT, LED_R_PIN);
    if (mask & 0x2) gpio_bits_reset(LED_G_GPIO_PORT, LED_G_PIN);
    else gpio_bits_set(LED_G_GPIO_PORT, LED_G_PIN);
    if (mask & 0x4) gpio_bits_reset(LED_B_GPIO_PORT, LED_B_PIN);
    else gpio_bits_set(LED_B_GPIO_PORT, LED_B_PIN);
}

void probe_led_init(void)
{
    memset(&led, 0, sizeof(led));
    apply(0, 0);
}

void probe_led_update(unsigned state)
{
    if (state > 6) return;
    if (state != led.state) {
        led.state = state;
        led.step = 0;
        apply(state, 0);
        if (steps[state][0].us) led.flip_us = probe_time_us() + steps[state][0].us;
        return;
    }
    if (steps[state][led.step].us && probe_time_us() >= led.flip_us) {
        led.step = (led.step + 1) % step_count[state];
        apply(state, led.step);
        led.flip_us += steps[state][led.step].us;
    }
}
