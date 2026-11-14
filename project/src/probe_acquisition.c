/**
 * @file probe_acquisition.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "at32f413_wk_config.h"
#include "probe_time.h"
#include "probe_acquisition.h"

/* Deep queues are what lets merged slots reach the wire caps: 4 slots of
 * small post-reset batches overflowed every few milliseconds at 4k/8k, and
 * each drop ended the segment before reads could merge (board round 13).
 * 8 raw slots absorb the bursts while the emission loop catches up; the 64KB
 * SRAM configuration pays for them (~13 KB). */
enum { QUEUE_SLOTS = 8, ATT_SLOTS = 4, CONFIG_STEPS = 40, SPI_TIMEOUT_US = 2000 };
/* ICM single-read cap. At 16k the old 512 B cap sat exactly on the
 * per-cycle production, so every read truncated at the balance point and
 * the FIFO crept to full at ~0.2% of the data rate (board round 21 RTT
 * diag: gap 3 ms, fhw 672 -> 2048 over seconds). One read now drains the
 * whole FIFO (1024 B = 64 samples = one full slot), removing the residual
 * the cap used to leave behind. */
enum { ICM_READ_CAP_BYTES = 1024 };
/* Consecutive continuous reads of one segment share a queue slot until the
 * slot hits this window's end or the wire batch cap. Fewer, larger frames
 * keep the shallow protocol queue out of overflow without slowing the FIFO
 * service cadence; 40 ms stays inside the live-UI latency budget. */
enum { MERGE_WINDOW_US = 40000 };
/* A queued batch becomes exactly one stream frame (32B batch header +
 * samples*per_sample must fit the frozen 1005B payload): the ADXL wire cap is
 * 108 samples, ICM 74. Each slot stores raw FIFO bytes though (ADXL 9B,
 * ICM 16B per sample) and data[] is 1024B, so the ICM slot limit is
 * 1024/16 = 64 samples. Single reads stay far below both caps (512B read
 * cap = 56 ADXL / 32 ICM samples). */
typedef enum {
    ACQ_OFF, ACQ_SELECT_BANK, ACQ_SELECT_BANK_WAIT, ACQ_ID, ACQ_ID_WAIT, ACQ_CONFIG, ACQ_CONFIG_WAIT,
    ACQ_VERIFY, ACQ_VERIFY_WAIT, ACQ_COUNT, ACQ_COUNT_WAIT, ACQ_TEMP_WAIT, ACQ_TEMP_CONFIRM_WAIT,
    ACQ_DATA_WAIT, ACQ_POST_COUNT_WAIT, ACQ_STOP_BANK, ACQ_STOP_BANK_WAIT,
    ACQ_STOP_POWER, ACQ_STOP_POWER_WAIT, ACQ_FAILED
} acquisition_state;

typedef struct {
    acquisition_state state;
    sensor_register_setting settings[CONFIG_STEPS];
    unsigned setting_count, setting;
    uint64_t due_us, warmup_until_us, last_data_us;
    uint64_t read_started_us, batch_irq_us, sequence;
    uint64_t count_started_us, batch_anchor_us; /* ICM FIFO-count observation, not watermark IRQ */
    uint32_t observed_irq, query_interval_us, odr_hz;
    uint64_t sensor_tick, batch_first_tick;
    uint16_t previous_tick;
    uint8_t tick_valid;
    uint8_t time_dirty; /* queue drops hide real time inside a segment:
                          * batches stay TIME_UNCERTAIN until the estimator
                          * re-anchors on a >=100 ms observation span */
    /* Timeline estimator (M5 frozen decisions). */
    uint8_t anchored, aligned, tick_anchored, tick_aligned;
    uint32_t period_q16, tick_period_q16;
    uint64_t anchor_seq, anchor_us, tick_anchor_us, tick_anchor_tick;
    uint16_t transfer_bytes;
    uint8_t pending[1024]; /* Kept while SPI reads post-transfer overflow status. */
    uint8_t backlog_read; /* Last read capped by the FIFO-read limit. */
    uint8_t head, queued;
    uint8_t att_head, att_queued;
    probe_acquisition_status status;
    probe_raw_batch queue[QUEUE_SLOTS];
    probe_raw_batch att_queue[ATT_SLOTS];
} acquisition_device;

typedef struct {
    uint32_t count;
    uint64_t time_us;
} acquisition_irq;

typedef struct {
    acquisition_device devices[PROBE_SPI_COUNT];
    volatile acquisition_irq irqs[3];
    uint8_t stopping, pose_only;
    probe_acquisition_status saved_icm;
    uint64_t last_poll_us;
    uint32_t max_poll_gap_us;
    probe_temperature temperature;
    uint64_t temperature_due;
    uint16_t temperature_candidate;
} acquisition_context;
static acquisition_context acquisition;

static uint8_t command(probe_spi_bus bus, uint8_t reg, int read)
{
    return bus == PROBE_SPI_ADXL ? adxl357_spi_command(reg, read) : icm42688_spi_command(reg, read);
}

static acquisition_irq irq_snapshot(unsigned source)
{
    acquisition_irq result;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    result.count = acquisition.irqs[source].count;
    result.time_us = acquisition.irqs[source].time_us;
    __set_PRIMASK(mask);
    return result;
}

void probe_acquisition_irq(unsigned source)
{
    if (source >= 3) return;
    acquisition.irqs[source].time_us = probe_time_us();
    ++acquisition.irqs[source].count;
}

static void end_segment(acquisition_device *device, int unknown_loss)
{
    ++device->status.segment;
    if (unknown_loss) ++device->status.unknown_gaps;
    device->sequence = 0;
    device->tick_valid = 0;
    /* Anchors are segment-local; the learned periods stay useful. */
    device->anchored = 0;
    device->aligned = 0;
    device->tick_anchored = 0;
    device->tick_aligned = 0;
    device->status.time_valid = 0;
    device->status.tick_valid = 0;
}

static void fault(acquisition_device *device, unsigned reason, uint64_t now)
{
    device->status.last_fault = reason;
    device->status.running = 0;
    end_segment(device, 1);
    if (reason == PROBE_ACQ_FAULT_SPI) ++device->status.spi_errors;
    if (reason == PROBE_ACQ_FAULT_OVERFLOW) ++device->status.overflows;
    if (reason == PROBE_ACQ_FAULT_FORMAT) ++device->status.format_errors;
    if (reason == PROBE_ACQ_FAULT_ID || reason == PROBE_ACQ_FAULT_CONFIG) {
        device->state = ACQ_FAILED;
    } else {
        /* Unknown FIFO loss: reset and reconfigure this chip only. Gyro is
         * allowed >= 45 ms on-time before the next reset/power transition. */
        device->setting = 0;
        device->state = ACQ_SELECT_BANK;
        device->due_us = now + 60000;
    }
}

static int read_registers(probe_spi_bus bus, uint8_t reg, unsigned bytes,
                          acquisition_state next, uint64_t now)
{
    acquisition_device *device = &acquisition.devices[bus];
    if (probe_spi_read(bus, command(bus, reg, 1), (uint16_t)bytes, SPI_TIMEOUT_US) != PROBE_SPI_PENDING) {
        fault(device, PROBE_ACQ_FAULT_SPI, now);
        return 0;
    }
    device->state = next;
    return 1;
}

static void write_register(probe_spi_bus bus, uint8_t reg, uint8_t value,
                           acquisition_state next, uint64_t now)
{
    uint8_t tx[2];
    tx[0] = command(bus, reg, 0);
    tx[1] = value;
    if (probe_spi_start(bus, tx, 2, SPI_TIMEOUT_US) != PROBE_SPI_PENDING)
        fault(&acquisition.devices[bus], PROBE_ACQ_FAULT_SPI, now);
    else {
        acquisition.devices[bus].state = next;
        if (bus == PROBE_SPI_ICM && reg == ICM42688_POWER_REG && value == 0x0F)
            acquisition.devices[bus].warmup_until_us = now + 60000;
    }
}

static int waiting(acquisition_state state)
{
    return state == ACQ_SELECT_BANK_WAIT || state == ACQ_ID_WAIT || state == ACQ_CONFIG_WAIT || state == ACQ_VERIFY_WAIT ||
           state == ACQ_COUNT_WAIT || state == ACQ_DATA_WAIT || state == ACQ_POST_COUNT_WAIT ||
           state == ACQ_STOP_BANK_WAIT || state == ACQ_STOP_POWER_WAIT ||
           state == ACQ_TEMP_WAIT || state == ACQ_TEMP_CONFIRM_WAIT;
}

static int validate_batch(probe_spi_bus bus, acquisition_device *device,
                          const uint8_t *data, uint16_t *flags)
{
    unsigned offset, axis;
    unsigned stride = bus == PROBE_SPI_ADXL ? 9 : 16;
    int32_t accel[3], gyro[3] = {0};
    icm42688_fifo_sample sample;
    for (offset = 0; offset < device->transfer_bytes; offset += stride) {
        if (bus == PROBE_SPI_ADXL) {
            if (!adxl357_decode_fifo_xyz(data + offset, stride, accel)) return 0;
        } else {
            if (!icm42688_decode_fifo(data + offset, stride, &sample)) return 0;
            /* An ODR-change bit during a running segment is a discontinuity. */
            if (sample.odr_changed && device->sequence) return 0;
            for (axis = 0; axis < 3; ++axis) {
                accel[axis] = sample.accel[axis];
                gyro[axis] = sample.gyro[axis];
            }
            if (device->tick_valid) {
                uint16_t delta = (uint16_t)(sample.timestamp - device->previous_tick);
                unsigned expected = 1000000U / device->odr_hz;
                if (!delta || delta < expected / 2 || delta > expected + expected / 2 + 2) return 0;
                device->sensor_tick += delta;
            } else {
                device->sensor_tick = sample.timestamp;
                device->tick_valid = 1;
            }
            device->previous_tick = sample.timestamp;
            if (!offset) device->batch_first_tick = device->sensor_tick;
            device->status.icm_timestamp = sample.timestamp;
        }
        for (axis = 0; axis < 3; ++axis) {
            int32_t limit = bus == PROBE_SPI_ADXL ? 498073 : 31129; /* 95% FS */
            if (accel[axis] >= limit || accel[axis] <= -limit ||
                gyro[axis] >= 31129 || gyro[axis] <= -31129)
                *flags |= PROBE_BATCH_NEAR_FULL_SCALE;
        }
    }
    memcpy(device->status.accel, accel, sizeof(accel));
    memcpy(device->status.gyro, gyro, sizeof(gyro));
    return 1;
}

static void accept_batch(probe_spi_bus bus, acquisition_device *device,
                         const uint8_t *data, uint64_t now)
{
    probe_raw_batch *batch;
    unsigned samples = device->transfer_bytes / (bus == PROBE_SPI_ADXL ? 9 : 16);
    unsigned batch_cap = bus == PROBE_SPI_ADXL ? PROBE_ADXL_BATCH_SAMPLES : PROBE_ICM_BATCH_SAMPLES;
    uint16_t flags = PROBE_BATCH_TIME_UNCERTAIN;
    uint64_t first_sample_us;
    int merged;
    device->last_data_us = now;
    if (now < device->warmup_until_us) {
        device->status.discarded_startup += samples;
        return;
    }
    if (!validate_batch(bus, device, data, &flags)) {
        fault(device, PROBE_ACQ_FAULT_FORMAT, now);
        return;
    }
    /* Merge this read into the newest queued slot when it continues the same
     * segment seamlessly; otherwise it opens a new slot. */
    batch = device->queued ?
            &device->queue[(device->head + device->queued - 1) % QUEUE_SLOTS] : 0;
    merged = !acquisition.pose_only && batch && batch->segment == device->status.segment &&
             batch->first_sequence + batch->samples == device->sequence &&
             batch->samples + samples <= batch_cap &&
             batch->size + device->transfer_bytes <= sizeof batch->data &&
             now - batch->read_started_us < MERGE_WINDOW_US;
    if (!acquisition.pose_only && !merged && device->queued == QUEUE_SLOTS) {
        /* Preserve already queued data; this fully read batch is dropped.
         * The segment survives: end_segment here used to shatter batches
         * into small segment-start frames exactly while the host lagged,
         * a feedback loop that throttled 16k to ~75% (board round 24).
         * The dropped samples occupy no sequence space, so the delivered
         * stream stays continuous; real time jumps by the dropped span,
         * which time_dirty covers with TIME_UNCERTAIN until the next
         * estimator re-anchor. */
        device->status.queue_drops += samples;
        device->status.last_fault = PROBE_ACQ_FAULT_QUEUE;
        device->time_dirty = 1;
        return;
    }
    /* Timeline update before queueing so the batch carries its estimate. */
    {
        uint64_t t = bus == PROBE_SPI_ICM ? device->batch_anchor_us : device->batch_irq_us;
        uint32_t span_us = bus == PROBE_SPI_ICM ? 100000U : 10000U;
        uint64_t seq_last = device->sequence + samples - 1;
        if (!device->anchored) {
            device->anchored = 1;
            device->anchor_seq = seq_last;
            device->anchor_us = t;
        } else if (t != device->anchor_us) {
            /* Measure over >=10 ms (DRDY) / >=100 ms (FIFO count): read cadence and anchor
             * cadence beat against each other, and short spans would turn
             * that beat into fake period jumps. Batches sharing one anchor
             * instant (or inside the span) keep the earlier anchor. */
            uint64_t seq_delta = seq_last - device->anchor_seq;
            if (seq_delta && t > device->anchor_us && t - device->anchor_us >= span_us) {
                uint64_t measured = ((t - device->anchor_us) << 16) / seq_delta;
                int64_t diff = (int64_t)measured - (int64_t)device->period_q16;
                if (diff < 0) diff = -diff;
                if (diff <= (int64_t)(device->period_q16 >> 4)) {
                    if (device->aligned < 255) ++device->aligned;
                } else {
                    device->aligned = 0;
                    device->period_q16 = (uint32_t)measured;
                }
                device->period_q16 =
                    (uint32_t)((int64_t)device->period_q16 +
                               (((int64_t)measured - (int64_t)device->period_q16) / 8));
                device->anchor_seq = seq_last;
                device->anchor_us = t;
                device->time_dirty = 0; /* real time re-anchored after drops */
            }
        }
        device->status.time_valid = device->aligned >= 8 && !device->time_dirty;
        if (device->aligned >= 8 && !device->time_dirty)
            flags &= (uint16_t)~PROBE_BATCH_TIME_UNCERTAIN;
        if (bus == PROBE_SPI_ICM) {
            if (!device->tick_anchored) {
                device->tick_anchored = 1;
                device->tick_anchor_us = t;
                device->tick_anchor_tick = device->sensor_tick;
            } else if (t > device->tick_anchor_us &&
                       t - device->tick_anchor_us >= span_us) {
                uint64_t tick_delta = device->sensor_tick - device->tick_anchor_tick;
                if (tick_delta) {
                    uint64_t measured = ((t - device->tick_anchor_us) << 16) / tick_delta;
                    int64_t diff = (int64_t)measured - (int64_t)device->tick_period_q16;
                    if (diff < 0) diff = -diff;
                    if (diff <= (int64_t)(device->tick_period_q16 >> 4)) {
                        if (device->tick_aligned < 255) ++device->tick_aligned;
                    } else {
                        device->tick_aligned = 0;
                        device->tick_period_q16 = (uint32_t)measured;
                    }
                    device->tick_period_q16 =
                        (uint32_t)((int64_t)device->tick_period_q16 +
                                   (((int64_t)measured - (int64_t)device->tick_period_q16) / 8));
                    device->tick_anchor_us = t;
                    device->tick_anchor_tick = device->sensor_tick;
                }
            }
            device->status.tick_valid = device->tick_aligned >= 8;
            device->status.tick_period_q16 = device->tick_period_q16;
        }
    }
    /* The retained anchor belongs to anchor_seq, not every new batch's end.
     * Project by sequence between estimator updates instead of repeating time. */
    if (device->sequence >= device->anchor_seq)
        first_sample_us = device->anchor_us +
            (((device->sequence - device->anchor_seq) * device->period_q16) >> 16);
    else {
        uint64_t back_us = ((device->anchor_seq - device->sequence) * device->period_q16) >> 16;
        first_sample_us = device->anchor_us > back_us ? device->anchor_us - back_us : 0;
    }
    if (merged) {
        memcpy(batch->data + batch->size, data, device->transfer_bytes);
        batch->size += device->transfer_bytes;
        batch->samples = (uint16_t)(batch->samples + samples);
        batch->last_sensor_tick = bus == PROBE_SPI_ICM ? device->sensor_tick : batch->last_sensor_tick;
        batch->flags = (uint16_t)(batch->flags | flags);
    } else {
        batch = &device->queue[(device->head + device->queued) % QUEUE_SLOTS];
        batch->period_q16 = device->period_q16;
        batch->first_sample_us = first_sample_us;
        batch->first_sequence = device->sequence;
        batch->segment = device->status.segment;
        batch->samples = (uint16_t)samples;
        batch->size = device->transfer_bytes;
        batch->read_started_us = device->read_started_us;
        batch->irq_time_us = device->batch_irq_us;
        batch->first_sensor_tick = bus == PROBE_SPI_ICM ? device->batch_first_tick : 0;
        batch->last_sensor_tick = bus == PROBE_SPI_ICM ? device->sensor_tick : 0;
        batch->flags = (uint16_t)(flags | (device->sequence == 0 ? PROBE_BATCH_SEGMENT_START : 0));
        memcpy(batch->data, data, batch->size);
        if (!acquisition.pose_only) ++device->queued;
        if (device->queued > device->status.queue_highwater) device->status.queue_highwater = device->queued;
    }
    device->status.queued_batches = (uint16_t)device->queued;
    if (!acquisition.pose_only) device->status.queued_bytes += device->transfer_bytes;
    if (bus == PROBE_SPI_ICM) {
        if (device->att_queued == ATT_SLOTS) {
            device->status.att_drops++;
        } else {
            probe_raw_batch *att = &device->att_queue[(device->att_head + device->att_queued) % ATT_SLOTS];
            /* Attitude consumes each read at sensor cadence, independent of
             * how raw reads were merged into wire batches. */
            *att = *batch;
            att->first_sequence = device->sequence;
            att->samples = (uint16_t)samples;
            att->size = device->transfer_bytes;
            att->first_sample_us = first_sample_us;
            att->read_started_us = device->read_started_us;
            att->irq_time_us = device->batch_irq_us;
            att->first_sensor_tick = device->batch_first_tick;
            att->last_sensor_tick = device->sensor_tick;
            att->flags = (uint16_t)(flags | (device->sequence == 0 ? PROBE_BATCH_SEGMENT_START : 0));
            att->period_q16 = device->period_q16;
            att->segment = device->status.segment;
            memcpy(att->data, data, att->size);
            ++device->att_queued;
        }
    }
    device->sequence += samples;
    device->status.last_sequence = device->sequence - 1;
    device->status.last_read_us = device->read_started_us;
    device->status.samples += samples;
    ++device->status.batches;
    device->status.running = 1;
}

static void poll_device(probe_spi_bus bus, uint64_t now)
{
    acquisition_device *device = &acquisition.devices[bus];
    const sensor_register_setting *step;
    const uint8_t *rx = 0;
    probe_spi_result result;
    acquisition_irq irq;
    unsigned bytes;
    if (device->state == ACQ_OFF) return;
    if (waiting(device->state)) {
        result = probe_spi_poll(bus);
        if (result == PROBE_SPI_PENDING) return;
        if (result != PROBE_SPI_DONE) {
            if (device->state == ACQ_TEMP_WAIT || device->state == ACQ_TEMP_CONFIRM_WAIT) {
                acquisition.temperature.valid = 0;
                device->state = ACQ_COUNT; /* Optional telemetry must not reset a raw segment. */
                return;
            }
            fault(device, PROBE_ACQ_FAULT_SPI, now);
            return;
        }
        rx = probe_spi_rx(bus) + 1;
    } else if (acquisition.stopping && now >= device->warmup_until_us &&
               device->state != ACQ_STOP_BANK && device->state != ACQ_STOP_POWER) {
        device->state = ACQ_STOP_BANK;
        device->due_us = now;
    }
    if (device->state == ACQ_FAILED ||
        (device->state != ACQ_COUNT && device->state != ACQ_TEMP_WAIT &&
         device->state != ACQ_TEMP_CONFIRM_WAIT && now < device->due_us)) return;
    step = &device->settings[device->setting];
    switch (device->state) {
    case ACQ_SELECT_BANK:
        if (bus == PROBE_SPI_ICM) write_register(bus, ICM42688_BANK_REG, 0, ACQ_SELECT_BANK_WAIT, now);
        else device->state = ACQ_ID;
        break;
    case ACQ_SELECT_BANK_WAIT:
        device->state = ACQ_ID;
        break;
    case ACQ_ID:
        read_registers(bus, bus == PROBE_SPI_ADXL ? ADXL357_ID_REG : ICM42688_ID_REG,
                       bus == PROBE_SPI_ADXL ? 4 : 1, ACQ_ID_WAIT, now);
        break;
    case ACQ_ID_WAIT:
        if (!(bus == PROBE_SPI_ADXL ? adxl357_identity_valid(rx, 4) : icm42688_identity_valid(rx, 1)))
            fault(device, PROBE_ACQ_FAULT_ID, now);
        else device->state = ACQ_CONFIG;
        break;
    case ACQ_CONFIG:
        if (device->setting == device->setting_count) {
            device->warmup_until_us = now + 60000;
            device->last_data_us = now;
            device->state = ACQ_COUNT;
        } else write_register(bus, step->reg, step->value, ACQ_CONFIG_WAIT, now);
        break;
    case ACQ_CONFIG_WAIT:
        device->due_us = now + step->wait_us;
        if (step->mask) device->state = ACQ_VERIFY;
        else { ++device->setting; device->state = ACQ_CONFIG; }
        break;
    case ACQ_VERIFY:
        read_registers(bus, step->reg, 1, ACQ_VERIFY_WAIT, now);
        break;
    case ACQ_VERIFY_WAIT:
        if ((rx[0] & step->mask) != (step->value & step->mask)) {
            device->status.fault_reg = step->reg;
            device->status.expected = step->value;
            device->status.actual = rx[0];
            fault(device, PROBE_ACQ_FAULT_CONFIG, now);
        }
        else { ++device->setting; device->state = ACQ_CONFIG; }
        break;
    case ACQ_COUNT:
        irq = irq_snapshot(bus == PROBE_SPI_ADXL ? 1 : 2);
        /* Pacing gate (see start()): batch_irq_us latches the newest edge,
         * the same anchor semantics as the old "any new edge" shortcut. */
        if (now < device->due_us) {
            if (bus == PROBE_SPI_ADXL && !acquisition.stopping &&
                now >= device->warmup_until_us && now >= acquisition.temperature_due) {
                acquisition.temperature_due = now + 1000000U;
                if (probe_spi_read(bus, adxl357_spi_command(6, 1), 2, SPI_TIMEOUT_US) == PROBE_SPI_PENDING)
                    device->state = ACQ_TEMP_WAIT;
                else acquisition.temperature.valid = 0;
            }
            break;
        }
        device->observed_irq = irq.count;
        device->batch_irq_us = bus == PROBE_SPI_ADXL ? irq_snapshot(0).time_us : irq.time_us;
        device->count_started_us = now;
        read_registers(bus, bus == PROBE_SPI_ADXL ? ADXL357_SNAPSHOT_REG : ICM42688_FIFO_STATUS_REG,
                       bus == PROBE_SPI_ADXL ? 2 : 3, ACQ_COUNT_WAIT, now);
        break;
    case ACQ_TEMP_WAIT:
        acquisition.temperature_candidate = (uint16_t)(((rx[0] & 15U) << 8) | rx[1]);
        /* TEMP2/TEMP1 are not double buffered: verify the high nibble after
         * the burst so a rollover cannot manufacture a 256-code jump. */
        if (probe_spi_read(bus, adxl357_spi_command(6, 1), 1, SPI_TIMEOUT_US) == PROBE_SPI_PENDING)
            device->state = ACQ_TEMP_CONFIRM_WAIT;
        else { acquisition.temperature.valid = 0; device->state = ACQ_COUNT; }
        break;
    case ACQ_TEMP_CONFIRM_WAIT:
        if ((rx[0] & 15U) == (acquisition.temperature_candidate >> 8) &&
            acquisition.temperature_candidate && acquisition.temperature_candidate < 4095U) {
            acquisition.temperature.raw = acquisition.temperature_candidate;
            acquisition.temperature.time_us = now;
            acquisition.temperature.valid = 1;
        } else acquisition.temperature.valid = 0;
        device->state = ACQ_COUNT;
        break;
    case ACQ_COUNT_WAIT:
        if (rx[0] & (bus == PROBE_SPI_ADXL ? ADXL357_STATUS_OVERFLOW : ICM42688_STATUS_FULL)) {
            fault(device, PROBE_ACQ_FAULT_OVERFLOW, now);
            break;
        }
        bytes = bus == PROBE_SPI_ADXL ? rx[1] * 3U : ((unsigned)rx[1] << 8) | rx[2];
        if (bytes > (bus == PROBE_SPI_ADXL ? 288U : ICM42688_FIFO_MAX_BYTES)) {
            fault(device, PROBE_ACQ_FAULT_FORMAT, now);
            break;
        }
        if (bytes > device->status.fifo_highwater) device->status.fifo_highwater = bytes;
        if (bus == PROBE_SPI_ICM) {
            /* FIFO_COUNT keeps advancing even when polling prevents watermark
             * IRQs. Midpoint approximates the register snapshot; the newest
             * sample is 0..one period old, use half a period. If >512 B is
             * queued, the batch we read ends before the newest FIFO sample. */
            uint64_t age_us = device->period_q16 >> 17;
            uint64_t observed_us = device->count_started_us + (now - device->count_started_us) / 2;
            if (bytes > ICM_READ_CAP_BYTES)
                age_us += ((uint64_t)((bytes - ICM_READ_CAP_BYTES) / 16) * device->period_q16) >> 16;
            device->batch_anchor_us = observed_us > age_us ? observed_us - age_us : 0;
        }
        /* A read capped by the FIFO-read limit means backlog: keep draining
         * back-to-back (see ACQ_POST_COUNT_WAIT) until it is gone. */
        device->backlog_read = bytes > (bus == PROBE_SPI_ADXL ? 288U : ICM_READ_CAP_BYTES);
        if (bus == PROBE_SPI_ICM && bytes > ICM_READ_CAP_BYTES) bytes = ICM_READ_CAP_BYTES;
        bytes -= bytes % (bus == PROBE_SPI_ADXL ? 9 : 16);
        if (!bytes) {
            if (now - device->last_data_us > 100000)
                fault(device, PROBE_ACQ_FAULT_NO_DATA, now);
            else { device->state = ACQ_COUNT; device->due_us = now + device->query_interval_us; }
        } else {
            device->transfer_bytes = (uint16_t)bytes;
            device->read_started_us = now;
            read_registers(bus, bus == PROBE_SPI_ADXL ? ADXL357_FIFO_REG : ICM42688_FIFO_REG,
                           bytes, ACQ_DATA_WAIT, now);
        }
        break;
    case ACQ_DATA_WAIT:
        memcpy(device->pending, rx, device->transfer_bytes);
        read_registers(bus, bus == PROBE_SPI_ADXL ? ADXL357_SNAPSHOT_REG : ICM42688_FIFO_STATUS_REG,
                       1, ACQ_POST_COUNT_WAIT, now);
        break;
    case ACQ_POST_COUNT_WAIT:
        if (rx[0] & (bus == PROBE_SPI_ADXL ? ADXL357_STATUS_OVERFLOW : ICM42688_STATUS_FULL)) {
            fault(device, PROBE_ACQ_FAULT_OVERFLOW, now);
            break;
        }
        device->state = ACQ_COUNT;
        /* Steady state paces reads (see start()); backlog drains immediately. */
        device->due_us = device->backlog_read ? now : now + device->query_interval_us;
        accept_batch(bus, device, device->pending, now);
        break;
    case ACQ_STOP_BANK:
        if (bus == PROBE_SPI_ICM) write_register(bus, ICM42688_BANK_REG, 0, ACQ_STOP_BANK_WAIT, now);
        else device->state = ACQ_STOP_POWER;
        break;
    case ACQ_STOP_BANK_WAIT:
        device->state = ACQ_STOP_POWER;
        break;
    case ACQ_STOP_POWER:
        write_register(bus, bus == PROBE_SPI_ADXL ? ADXL357_POWER_REG : ICM42688_POWER_REG,
                       bus == PROBE_SPI_ADXL ? 1 : 0, ACQ_STOP_POWER_WAIT, now);
        break;
    case ACQ_STOP_POWER_WAIT:
        device->state = ACQ_OFF;
        device->status.running = 0;
        device->status.stopped = 1;
        break;
    default: break;
    }
}

void probe_acquisition_init(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    memset(&acquisition, 0, sizeof(acquisition));
    acquisition.devices[0].status.stopped = 1;
    acquisition.devices[1].status.stopped = 1;
    exint_flag_clear(EXINT_LINE_0 | EXINT_LINE_1 | EXINT_LINE_10);
    nvic_irq_enable(EXINT0_IRQn, 3, 0);
    nvic_irq_enable(EXINT1_IRQn, 3, 0);
    nvic_irq_enable(EXINT15_10_IRQn, 3, 0);
    __set_PRIMASK(mask);
}

static int acquisition_start(const probe_acquisition_config *config, int pose_only)
{
    unsigned bus, counts[2];
    if (!config || config->adxl.hpf) return 0;
    for (bus = 0; bus < PROBE_SPI_COUNT; ++bus)
        if (acquisition.devices[bus].state != ACQ_OFF) return 0;
    counts[0] = adxl357_build_fifo_config(&config->adxl, acquisition.devices[0].settings, CONFIG_STEPS);
    counts[1] = icm42688_build_fifo_config(&config->icm, acquisition.devices[1].settings, CONFIG_STEPS);
    if (!counts[0] || !counts[1]) return 0;
    acquisition.stopping = 0;
    if (!pose_only) {
        acquisition.temperature.valid = 0;
        acquisition.temperature_due = 0;
    }
    if (pose_only) acquisition.saved_icm = acquisition.devices[PROBE_SPI_ICM].status;
    acquisition.pose_only = (uint8_t)pose_only;
    for (bus = 0; bus < PROBE_SPI_COUNT; ++bus) {
        acquisition_device *device = &acquisition.devices[bus];
        if (pose_only && bus == PROBE_SPI_ADXL) continue;
        memset(&device->status, 0, sizeof(device->status));
        device->status.segment = 1;
        device->head = device->queued = 0;
        device->att_head = device->att_queued = 0;
        device->sequence = device->setting = 0;
        device->tick_valid = 0;
        device->odr_hz = bus == PROBE_SPI_ADXL ? config->adxl.odr_hz : config->icm.odr_hz;
        device->query_interval_us = device->odr_hz < 500 ? 1000000U / device->odr_hz : 2000;
        /* Fast ICM ODR pacing. Board rounds 18-20 falsified every tighter
         * guess at 16k on real silicon (exact/half/quarter production
         * period all overflow-churned segments while the emulator passed),
         * so the interval returns to 32 MB/s / odr - the depth the PC CDC
         * link ran 16k with and the TAB5 acceptance ran 8k at (ADXL/ICM
         * 100%, zero ovf). The 2 ms interval at 16k relies on the backlog
         * catch-up for the >512 B bursts; if 16k still churns, the RTT
         * diag line (max_poll_gap / fifo high-water) discriminates a main
         * loop stall from a slow read pipeline instead of another pacing
         * guess. ADXL keeps the short interval: its 288 B FIFO fill is
         * small either way and it never saturated the queue. */
        if (bus == PROBE_SPI_ICM && device->odr_hz >= 4000)
            device->query_interval_us = 32000000U / device->odr_hz;
        /* 16k: shape reads to ~48 samples (3 ms) so the wire sees ~333 fps
         * of one-read frames - inside the TAB5 host's completion budget
         * (~340 fps measured with two different drivers). FIFO safe: the
         * 3.4 ms cycle accumulates ~870 B, under the 1024 B read cap. */
        if (bus == PROBE_SPI_ICM && device->odr_hz == 16000)
            device->query_interval_us = 3000;
        /* New configuration: timeline restarts from the nominal period. */
        device->anchored = device->aligned = device->tick_anchored = device->tick_aligned = 0;
        device->time_dirty = 0;
        device->period_q16 = (uint32_t)(((uint64_t)1000000 << 16) / device->odr_hz);
        device->tick_period_q16 = 65536U;
        device->setting_count = counts[bus];
        device->state = ACQ_SELECT_BANK;
        device->due_us = probe_time_us() + 100000;
    }
    return 1;
}

int probe_acquisition_start(const probe_acquisition_config *config)
{
    if (acquisition.pose_only) return 0;
    return acquisition_start(config, 0);
}

int probe_acquisition_pose_start(const icm42688_config *config)
{
    probe_acquisition_config pose = {{250, 10, 0}, {200, 16, 500}};
    if (!config || acquisition.pose_only || acquisition.devices[0].queued ||
        acquisition.devices[1].queued) return 0;
    pose.icm = *config;
    return acquisition_start(&pose, 1);
}
int probe_acquisition_pose_active(void) { return acquisition.pose_only; }

void probe_acquisition_poll(void)
{
    uint64_t now = probe_time_us();
    if (acquisition.last_poll_us && now - acquisition.last_poll_us > acquisition.max_poll_gap_us)
        acquisition.max_poll_gap_us = (uint32_t)(now - acquisition.last_poll_us);
    acquisition.last_poll_us = now;
    poll_device(PROBE_SPI_ADXL, now);
    poll_device(PROBE_SPI_ICM, now);
    if (acquisition.pose_only && acquisition.devices[PROBE_SPI_ICM].state == ACQ_OFF) {
        acquisition.devices[PROBE_SPI_ICM].status = acquisition.saved_icm;
        acquisition.devices[PROBE_SPI_ICM].att_head = acquisition.devices[PROBE_SPI_ICM].att_queued = 0;
        acquisition.pose_only = 0;
    }
}
void probe_acquisition_stop(void) { acquisition.stopping = 1; }

void probe_acquisition_get_temperature(probe_temperature *out)
{
    if (!out) return;
    *out = acquisition.temperature;
    out->live = (uint8_t)(out->valid && !acquisition.stopping &&
        acquisition.devices[PROBE_SPI_ADXL].status.running &&
        probe_time_us() - out->time_us < 2000000U);
}

void probe_acquisition_get_status(probe_spi_bus bus, probe_acquisition_status *status)
{
    if ((unsigned)bus >= PROBE_SPI_COUNT || !status) return;
    *status = acquisition.pose_only && bus == PROBE_SPI_ICM ?
              acquisition.saved_icm : acquisition.devices[bus].status;
    status->max_poll_gap_us = acquisition.max_poll_gap_us;
    status->irq_count = irq_snapshot(bus == PROBE_SPI_ADXL ? 0 : 2).count;
}

const probe_raw_batch *probe_acquisition_peek(probe_spi_bus bus)
{
    acquisition_device *device;
    if ((unsigned)bus >= PROBE_SPI_COUNT) return 0;
    device = &acquisition.devices[bus];
    return device->queued ? &device->queue[device->head] : 0;
}
void probe_acquisition_release(probe_spi_bus bus)
{
    acquisition_device *device;
    if ((unsigned)bus >= PROBE_SPI_COUNT) return;
    device = &acquisition.devices[bus];
    if (!device->queued) return;
    device->head = (device->head + 1) % QUEUE_SLOTS;
    --device->queued;
    device->status.queued_batches = (uint16_t)device->queued;
    device->status.queued_bytes -= device->queue[(device->head + QUEUE_SLOTS - 1) % QUEUE_SLOTS].size;
}

const probe_raw_batch *probe_acquisition_attitude_peek(void)
{
    acquisition_device *device = &acquisition.devices[PROBE_SPI_ICM];
    return device->att_queued ? &device->att_queue[device->att_head] : 0;
}

void probe_acquisition_attitude_release(void)
{
    acquisition_device *device = &acquisition.devices[PROBE_SPI_ICM];
    if (!device->att_queued) return;
    device->att_head = (device->att_head + 1) % ATT_SLOTS;
    --device->att_queued;
}
