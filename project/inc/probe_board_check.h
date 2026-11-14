/**
 * @file probe_board_check.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_BOARD_CHECK_H
#define PROBE_BOARD_CHECK_H
/* Mode 3 only: main-loop owner of acquisition, attitude, LED and RTT input.
 * Never run concurrently with probe_proto or the M2/M3 consumers. */
void probe_board_check_init(void);
void probe_board_check_poll(void);
#endif
