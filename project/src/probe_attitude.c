/**
 * @file probe_attitude.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "invn_algo_agm.h"
#include "probe_acquisition.h"
#include "probe_attitude.h"
#include "probe_attitude_filters.h"

/* AGM is deliberately fed at one rate (200 Hz).  Rates >= 1 kHz are reduced
 * by 2:1 FIR stages to 1 kHz and then 5:1; 500 Hz is a 2:5 rational
 * resampler implemented as upsample-by-two, FIR, decimate-by-five. */

#define ATT_HALF_TAPS 63U
#define ATT_FINAL_TAPS 99U
#define ATT_MAX_HALF_STAGES 5U
#define ATT_LIB_SCALE 16
#define ATT_PACKETS_PER_BATCH (512U / ICM42688_FIFO_PACKET_BYTES)
#define ATT_OUT_PERIOD_US 5000U
#define ATT_OUTPUT_QUEUE_CAPACITY 8U

typedef enum {
    ATT_FINAL_DECIMATE_5,
    ATT_FINAL_RATIONAL_2_5,
    ATT_FINAL_DIRECT
} att_final_mode;

typedef struct {
    uint8_t active;
    uint8_t half_stages;
    uint8_t final_divisor;
    att_final_mode final_mode;
    uint32_t delay_source_samples_q16;
    icm42688_config config;
    int16_t half_line[ATT_MAX_HALF_STAGES][6][ATT_HALF_TAPS];
    int16_t final_line[6][ATT_FINAL_TAPS];
    uint32_t half_in[ATT_MAX_HALF_STAGES];
    uint32_t final_in;
    InvnAlgoAGMConfig agm_config;
    InvnAlgoAGMInput input;
    InvnAlgoAGMOutput output;
    icm42688_fifo_sample scratch[ATT_PACKETS_PER_BATCH];
    probe_attitude_sample outputs[ATT_OUTPUT_QUEUE_CAPACITY];
    uint8_t output_head, output_tail, output_count, latest_only;
    uint64_t next_sequence;
    probe_attitude_status status;
} att_context;

static att_context att;

static void stage_push(int16_t *line, unsigned taps, int16_t sample)
{
    memmove(line, line + 1, (taps - 1) * sizeof(int16_t));
    line[taps - 1] = sample;
}

static void stage_dot(const int16_t *line, const float *coef, unsigned taps,
                      float scale, int16_t *out6)
{
    unsigned ch, i;
    for (ch = 0; ch < 6; ++ch) {
        float sum = 0;
        for (i = 0; i < taps; ++i) sum += (float)line[ch * taps + i] * coef[i];
        out6[ch] = (int16_t)(sum * scale);
    }
}

static void attitude_emit(const int16_t *sample, int8_t temperature,
                          uint64_t time_q16, uint32_t source_period_q16,
                          uint16_t batch_flags)
{
    unsigned axis;
    uint64_t output_time_us;
    for (axis = 0; axis < 3; ++axis) {
        att.status.last_output[axis] = sample[axis];
        att.status.last_output[axis + 3] = sample[axis + 3];
        att.input.sRacc_data[axis] = (int32_t)sample[axis] * ATT_LIB_SCALE;
        att.input.sRgyr_data[axis] = (int32_t)sample[axis + 3] * ATT_LIB_SCALE;
    }
    att.input.sRtemp_data = (int16_t)temperature;
    att.input.mask = INVN_ALGO_AGM_INPUT_MASK_ACC | INVN_ALGO_AGM_INPUT_MASK_GYR;
    time_q16 -= ((uint64_t)source_period_q16 * att.delay_source_samples_q16) >> 16;
    output_time_us = time_q16 >> 16;
    att.input.sRimu_time_us = (int64_t)output_time_us;
    invn_algo_agm_process(&att.input, &att.output);
    ++att.status.resampled;
    if (att.output.mask & INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AG) {
        for (axis = 0; axis < 4; ++axis)
            att.status.quat_q30[axis] = att.output.grv_quat_q30[axis];
        att.status.quat_valid = 1;
        ++att.status.processed;
        if (att.latest_only && att.output_count) {
            att.status.dropped_outputs += att.output_count;
            att.output_tail = att.output_head;
            att.output_count = 0;
        }
        if (att.output_count < ATT_OUTPUT_QUEUE_CAPACITY) {
            probe_attitude_sample *queued = &att.outputs[att.output_head];
            queued->sequence = att.next_sequence;
            queued->time_us = output_time_us;
            memcpy(queued->quat_q30, att.status.quat_q30, sizeof(queued->quat_q30));
            queued->flags = (uint16_t)(batch_flags & PROBE_BATCH_TIME_UNCERTAIN);
            att.output_head = (uint8_t)((att.output_head + 1U) % ATT_OUTPUT_QUEUE_CAPACITY);
            ++att.output_count;
            att.status.queued_outputs = att.output_count;
        } else {
            ++att.status.dropped_outputs;
        }
        ++att.next_sequence;
    }
}

static void final_tick(const int16_t *sample, int8_t temperature,
                       uint64_t time_q16, uint32_t source_period_q16,
                       uint16_t batch_flags)
{
    unsigned axis;
    int16_t output[6];
    const float *coef = att.final_mode == ATT_FINAL_DIRECT
                            ? attitude_fir_200 : attitude_fir_1k;
    const float scale = att.final_mode == ATT_FINAL_RATIONAL_2_5 ? 2.0f : 1.0f;

    for (axis = 0; axis < 6; ++axis)
        stage_push(att.final_line[axis], ATT_FINAL_TAPS, sample[axis]);
    ++att.final_in;
    if (att.final_in < ATT_FINAL_TAPS || att.final_in % att.final_divisor) return;
    stage_dot(&att.final_line[0][0], coef, ATT_FINAL_TAPS, scale, output);
    attitude_emit(output, temperature, time_q16, source_period_q16, batch_flags);
}

static void final_push(const int16_t *sample, int8_t temperature,
                       uint64_t time_q16, uint32_t source_period_q16,
                       uint16_t batch_flags)
{
    static const int16_t zero[6];
    final_tick(sample, temperature, time_q16, source_period_q16, batch_flags);
    if (att.final_mode == ATT_FINAL_RATIONAL_2_5)
        final_tick(zero, temperature, time_q16 + source_period_q16 / 2U,
                   source_period_q16, batch_flags);
}

static void half_push(unsigned stage, const int16_t *sample, int8_t temperature,
                      uint64_t time_q16, uint32_t source_period_q16,
                      uint16_t batch_flags)
{
    unsigned axis;
    int16_t output[6];

    for (axis = 0; axis < 6; ++axis)
        stage_push(att.half_line[stage][axis], ATT_HALF_TAPS, sample[axis]);
    ++att.half_in[stage];
    if (att.half_in[stage] < ATT_HALF_TAPS || (att.half_in[stage] & 1U)) return;
    stage_dot(&att.half_line[stage][0][0], attitude_fir_half, ATT_HALF_TAPS,
              1.0f, output);
    if (stage + 1U < att.half_stages)
        half_push(stage + 1U, output, temperature, time_q16, source_period_q16,
                  batch_flags);
    else
        final_push(output, temperature, time_q16, source_period_q16, batch_flags);
}

static int attitude_select_rate(uint32_t odr_hz)
{
    att.half_stages = 0;
    att.final_mode = ATT_FINAL_DECIMATE_5;
    att.final_divisor = 5;
    switch (odr_hz) {
    case 32000U: att.half_stages = 5; break;
    case 16000U: att.half_stages = 4; break;
    case 8000U:  att.half_stages = 3; break;
    case 4000U:  att.half_stages = 2; break;
    case 2000U:  att.half_stages = 1; break;
    case 1000U:  break;
    case 500U:
        att.final_mode = ATT_FINAL_RATIONAL_2_5;
        break;
    case 200U:
        att.final_mode = ATT_FINAL_DIRECT;
        att.final_divisor = 1;
        break;
    default:
        return 0;
    }
    if (att.final_mode == ATT_FINAL_RATIONAL_2_5)
        att.delay_source_samples_q16 = 49U << 15;
    else if (att.final_mode == ATT_FINAL_DIRECT)
        att.delay_source_samples_q16 = 49U << 16;
    else
        att.delay_source_samples_q16 = (31U * ((1U << att.half_stages) - 1U)
                                        + 49U * (1U << att.half_stages)) << 16;
    return 1;
}

void probe_attitude_init(void)
{
    memset(&att, 0, sizeof(att));
}

int probe_attitude_start(const icm42688_config *config)
{
    att.active = 0;
    memset(&att, 0, sizeof(att));
    if (!config || !attitude_select_rate(config->odr_hz)) return 0;
    att.config = *config;
    att.agm_config.acc_fsr = config->accel_g;
    att.agm_config.gyr_fsr = config->gyro_dps;
    att.agm_config.acc_odr_us = ATT_OUT_PERIOD_US;
    att.agm_config.gyr_odr_us = ATT_OUT_PERIOD_US;
    att.agm_config.temp_offset = 25 * 65536;
    att.agm_config.temp_sensitivity = (int32_t)((100LL << 30) / 207);
    if (invn_algo_agm_init(&att.agm_config)) return 0;
    att.active = 1;
    return 1;
}

void probe_attitude_latest_only(int enabled) { att.latest_only = enabled != 0; }

void probe_attitude_stop(void)
{
    att.active = 0;
    att.status.quat_valid = 0;
}

void probe_attitude_ingest(const icm42688_fifo_sample *samples, unsigned count,
                           uint64_t first_sample_us, uint32_t period_q16,
                           uint16_t batch_flags)
{
    unsigned i, axis;
    if (!att.active || !count) return;
    for (i = 0; i < count; ++i) {
        int16_t raw[6];
        uint64_t time_q16 = (first_sample_us << 16) + (uint64_t)i * period_q16;
        for (axis = 0; axis < 3; ++axis) {
            raw[axis] = samples[i].accel[axis];
            raw[axis + 3] = samples[i].gyro[axis];
        }
        if (att.half_stages)
            half_push(0, raw, samples[i].temperature, time_q16, period_q16,
                      batch_flags);
        else
            final_push(raw, samples[i].temperature, time_q16, period_q16,
                       batch_flags);
    }
    att.status.samples_in += count;
}

void probe_attitude_poll(void)
{
    const probe_raw_batch *batch = probe_acquisition_attitude_peek();
    unsigned n, count = 0;
    if (!batch) return;
    if (!att.active) { probe_acquisition_attitude_release(); return; }
    for (n = 0; n + ICM42688_FIFO_PACKET_BYTES <= batch->size; n += ICM42688_FIFO_PACKET_BYTES) {
        if (count >= ATT_PACKETS_PER_BATCH) break;
        if (icm42688_decode_fifo(batch->data + n, ICM42688_FIFO_PACKET_BYTES,
                                 &att.scratch[count]))
            ++count;
    }
    if (count)
        probe_attitude_ingest(att.scratch, count, batch->first_sample_us,
                              batch->period_q16, batch->flags);
    else
        ++att.status.dropped_batches;
    probe_acquisition_attitude_release();
}

void probe_attitude_get_status(probe_attitude_status *status)
{
    if (status) *status = att.status;
}

const probe_attitude_sample *probe_attitude_peek(void)
{
    return att.output_count ? &att.outputs[att.output_tail] : 0;
}

void probe_attitude_release(void)
{
    if (!att.output_count) return;
    att.output_tail = (uint8_t)((att.output_tail + 1U) % ATT_OUTPUT_QUEUE_CAPACITY);
    --att.output_count;
    att.status.queued_outputs = att.output_count;
}
