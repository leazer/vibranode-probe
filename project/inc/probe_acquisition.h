/**
 * @file probe_acquisition.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_ACQUISITION_H
#define PROBE_ACQUISITION_H

#include "adxl357.h"
#include "icm42688.h"
#include "probe_spi.h"

typedef struct {
    adxl357_config adxl;
    icm42688_config icm;
} probe_acquisition_config;

enum {
    PROBE_BATCH_TIME_UNCERTAIN = 1,
    PROBE_BATCH_SEGMENT_START = 2,
    PROBE_BATCH_NEAR_FULL_SCALE = 4
};
/* Max samples one queued batch carries = one stream frame (32B batch header
 * + samples*per_sample must fit the frozen 1005B payload). ADXL 9B/sample;
 * ICM slots store raw 16B FIFO packets and data[] is 1024B -> 64. */
enum { PROBE_ADXL_BATCH_SAMPLES = 108, PROBE_ICM_BATCH_SAMPLES = 64 };
enum {
    PROBE_ACQ_FAULT_NONE,
    PROBE_ACQ_FAULT_SPI,
    PROBE_ACQ_FAULT_ID,
    PROBE_ACQ_FAULT_CONFIG,
    PROBE_ACQ_FAULT_OVERFLOW,
    PROBE_ACQ_FAULT_FORMAT,
    PROBE_ACQ_FAULT_QUEUE,
    PROBE_ACQ_FAULT_NO_DATA
};

typedef struct {
    uint64_t first_sequence;
    uint64_t read_started_us;
    uint64_t irq_time_us;
    uint64_t first_sensor_tick; /* ICM only; uncalibrated, segment-local */
    uint64_t last_sensor_tick;
    /* Timeline estimate (M5): ADXL DRDY / ICM FIFO-count observation;
     * irq_time_us above remains the physical IRQ diagnostic, not ICM model time.
     * TIME_UNCERTAIN is cleared only
     * while the per-device estimator reports a consistent period. */
    uint64_t first_sample_us;
    uint32_t period_q16;
    uint32_t segment;
    uint16_t samples;
    uint16_t size;
    uint16_t flags;
    uint8_t data[1024]; /* Original FIFO bytes; ICM includes header/time.
                         * Sized for one max wire batch (see probe_acquisition.c). */
} probe_raw_batch;

typedef struct {
    uint32_t samples, batches, discarded_startup;
    uint32_t spi_errors, overflows, format_errors, queue_drops;
    uint32_t segment, unknown_gaps, fifo_highwater, queue_highwater;
    uint32_t irq_count, last_fault, max_poll_gap_us;
    uint16_t queued_batches;
    uint32_t queued_bytes;
    int32_t accel[3], gyro[3];
    uint16_t icm_timestamp;
    uint8_t running, stopped;
    uint8_t fault_reg, expected, actual;
    /* Protocol STOP boundary: last sample handed to the raw queue. The read
     * time is a delivery anchor, not the sample's true timestamp. */
    uint64_t last_sequence, last_read_us;
    /* Timeline estimator state; tick_period_q16 is MCU us per sensor tick. */
    uint8_t time_valid, tick_valid;
    uint32_t tick_period_q16;
    uint32_t att_drops; /* attitude bypass full: batches not handed off */
} probe_acquisition_status;

/* Single owner: main loop. init/start require buses not owned by M2 diagnostics.
 * start returns zero if busy/invalid; it does not silently alter the config.
 * HPF codes are supported by chip configuration but runtime currently requires
 * HPF off until its long settling transient has a validated discard policy. */
void probe_acquisition_init(void);
int probe_acquisition_start(const probe_acquisition_config *config);
/* Private standby service: only ICM runs; raw queues and measurement status
 * remain frozen. Requires both hardware OFF and raw queues drained. stop()
 * quiesces asynchronously; pose_active stays true until hardware is OFF. */
int probe_acquisition_pose_start(const icm42688_config *config);
int probe_acquisition_pose_active(void);
void probe_acquisition_poll(void);
/* Diagnostic abort: waits for an in-flight transfer, powers down both chips.
 * Hardware FIFO tail is discarded. This is NOT the future wire STOP command. */
void probe_acquisition_stop(void);
void probe_acquisition_irq(unsigned source);
void probe_acquisition_get_status(probe_spi_bus bus, probe_acquisition_status *status);
typedef struct {
    uint64_t time_us;
    uint16_t raw;
    uint8_t valid, live;
} probe_temperature;
/* Cached ADXL die temperature. STOP keeps the last value, live becomes zero. */
void probe_acquisition_get_temperature(probe_temperature *temperature);
/* Main-loop borrow until release; do not retain pointer across start/init. */
const probe_raw_batch *probe_acquisition_peek(probe_spi_bus bus);
void probe_acquisition_release(probe_spi_bus bus);
/* ICM-only attitude bypass: independent bounded queue, drops when full. */
const probe_raw_batch *probe_acquisition_attitude_peek(void);
void probe_acquisition_attitude_release(void);

#endif
