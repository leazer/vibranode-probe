/**
 * @file probe_capture_check.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_CAPTURE_CHECK_H
#define PROBE_CAPTURE_CHECK_H
/* M3 diagnostic consumer of the acquisition API. No direct sensor transport. */
void probe_capture_check_init(void);
void probe_capture_check_poll(void);
#endif
