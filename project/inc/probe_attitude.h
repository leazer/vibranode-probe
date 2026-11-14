/**
 * @file probe_attitude.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_ATTITUDE_H
#define PROBE_ATTITUDE_H
#include <stdint.h>
#include "icm42688.h"

/* M7 attitude branch (probe design section 6): ICM raw samples -> FIR
 * resampler (all supported ODRs -> 200 Hz) -> TDK AGM six-axis game rotation
 * vector. Outputs use a bounded queue consumed by the protocol FETCH window.
 * At 200 SPS, source aliases above Nyquist cannot be undone. */

typedef struct {
    uint32_t samples_in, resampled, processed, dropped_batches, dropped_outputs;
    uint16_t queued_outputs;
    int16_t last_output[6]; /* latest 200 Hz filtered sample (raw counts) */
    int32_t quat_q30[4];    /* latest AGM game rotation vector */
    uint8_t quat_valid;
} probe_attitude_status;

typedef struct {
    uint64_t sequence;
    uint64_t time_us;
    int32_t quat_q30[4];
    uint16_t flags; /* bit0: source time uncertain */
} probe_attitude_sample;

void probe_attitude_init(void);
/* (Re)start for a supported ICM configuration (200 through 32000 SPS). */
int probe_attitude_start(const icm42688_config *config);
void probe_attitude_stop(void);
/* Standby UI mailbox: replace older unsent outputs with the latest pose.
 * start() restores the normal bounded measurement FIFO policy. */
void probe_attitude_latest_only(int enabled);
/* Feed decoded FIFO samples of one batch (main loop, bounded). */
void probe_attitude_ingest(const icm42688_fifo_sample *samples, unsigned count,
                           uint64_t first_sample_us, uint32_t period_q16,
                           uint16_t batch_flags);
/* Main-loop service: consume decoded ICM batches from the acquisition tap. */
void probe_attitude_poll(void);
void probe_attitude_get_status(probe_attitude_status *status);
const probe_attitude_sample *probe_attitude_peek(void);
void probe_attitude_release(void);

#endif
