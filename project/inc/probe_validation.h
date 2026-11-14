/**
 * @file probe_validation.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_VALIDATION_H
#define PROBE_VALIDATION_H
/* Build mode: 0 = production RS-485 protocol app, 1 = M2 board diagnostics,
 * 2 = M3 dual FIFO capture, 3 = M4-M8 single-board RTT validation.
 * Only mode 0 streams on the wire. Current bring-up priority: mode 3. */
#ifndef PROBE_HARDWARE_VALIDATION
#define PROBE_HARDWARE_VALIDATION 0
#endif
void probe_validation_init(void);
void probe_validation_poll(void);
/* 0: ADXL DRDY/PB0, 1: ADXL INT1/PB1, 2: ICM INT1/PB10. */
void probe_validation_irq(unsigned source);
#endif
