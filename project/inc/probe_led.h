/**
 * @file probe_led.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_LED_H
#define PROBE_LED_H

/* Status LED state machine (probe design section 9). The state value is the
 * protocol work-state enum (0-5, resolved upstream and mirrored on the wire)
 * plus the firmware-local state 6 = "no USB host", which the production poll
 * substitutes while probe_usb_connected() is false and which therefore never
 * appears in a frame. The LED only maps state -> colour pattern:
 *   0 blue steady    1 blue 2 s heartbeat 100/1900   2 green 1 Hz double
 *   pulse 100/50/100/750    3 purple 5 Hz    4 red 5 Hz    5 red steady
 *   6 white 1 Hz 500/500
 * A state switch always starts from the first (lit) phase. */
void probe_led_init(void);
void probe_led_update(unsigned state);

#endif
