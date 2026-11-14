/**
 * @file probe_calib.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include <math.h>
#include "at32f413_wk_config.h"
#include "probe_frame.h" /* CRC-32C */
#include "at32f413_flash.h"
#include "probe_calib.h"

/* Last two 2 KB sectors of the 256 KB flash; one record per sector. */
#define CALIB_SECTOR_BASE 0x0803F000U
#define CALIB_SECTOR_SIZE 2048U
#define CALIB_RECORD_PAYLOAD (sizeof(probe_calib_record) - sizeof(uint32_t))

typedef struct {
    uint32_t magic;
    uint8_t version, valid_flags;
    uint16_t reserved;
    float accel_bias_g[3], accel_scale[3], gyro_bias_dps[3], temp_c;
    uint32_t created_ms, generation, crc;
} probe_calib_record_v2;

typedef struct {
    probe_calib_record current;
    probe_calib_record candidate;
    probe_calib_status status;
    double faces[2][6][3];
    double sum[6], square[6], temp_sum;
    double accel_counts_per_g, gyro_counts_per_dps;
    uint64_t deadline_us, next_sequence;
    uint32_t capture_segment;
    uint8_t face_mask[2], sequence_valid;
} probe_calib_context;

static probe_calib_context calib;

static const probe_calib_record *record_at(unsigned slot)
{
    return (const probe_calib_record *)(CALIB_SECTOR_BASE + slot * CALIB_SECTOR_SIZE);
}

static int record_valid(const probe_calib_record *r)
{
    uint32_t crc;
    if (r->magic != PROBE_CALIB_MAGIC || r->version != PROBE_CALIB_VERSION) return 0;
    if (r->valid_flags & ~((uint8_t)(PROBE_CALIB_VALID_ACCEL | PROBE_CALIB_VALID_GYRO |
                                     PROBE_CALIB_VALID_ICM_ACCEL))) return 0;
    crc = probe_frame_crc32c((const uint8_t *)r, CALIB_RECORD_PAYLOAD);
    return crc == r->crc;
}

static int newer(const probe_calib_record *a, const probe_calib_record *b)
{
    return (int32_t)(a->generation - b->generation) > 0;
}

static void defaults(probe_calib_record *r)
{
    unsigned i;
    memset(r, 0, sizeof(*r));
    r->magic = PROBE_CALIB_MAGIC;
    r->version = PROBE_CALIB_VERSION;
    for (i = 0; i < 3; ++i) {
        r->accel_scale[i] = 1.0f;
        r->icm_accel_scale[i] = 1.0f;
    }
}

static int load_slot(unsigned slot, probe_calib_record *out)
{
    const probe_calib_record *current = record_at(slot);
    const probe_calib_record_v2 *old = (const probe_calib_record_v2 *)current;
    if (record_valid(current)) { *out = *current; return 1; }
    if (old->magic == PROBE_CALIB_MAGIC && old->version == 2U &&
        !(old->valid_flags & ~(PROBE_CALIB_VALID_ACCEL | PROBE_CALIB_VALID_GYRO)) &&
        probe_frame_crc32c((const uint8_t *)old, sizeof(*old) - sizeof(uint32_t)) == old->crc) {
        defaults(out);
        out->valid_flags = old->valid_flags;
        memcpy(out->accel_bias_g, old->accel_bias_g, sizeof(old->accel_bias_g));
        memcpy(out->accel_scale, old->accel_scale, sizeof(old->accel_scale));
        memcpy(out->gyro_bias_dps, old->gyro_bias_dps, sizeof(old->gyro_bias_dps));
        out->temp_c = old->temp_c;
        out->created_ms = old->created_ms;
        out->generation = old->generation;
        return 1;
    }
    return 0;
}

void probe_calib_init(void)
{
    probe_calib_record a, b;
    int va, vb;
    memset(&calib, 0, sizeof(calib));
    va = load_slot(0, &a); vb = load_slot(1, &b);
    if (va && vb)
        calib.current = newer(&b, &a) ? b : a;
    else if (va)
        calib.current = a;
    else if (vb)
        calib.current = b;
    else
        defaults(&calib.current);
    calib.candidate = calib.current;
    calib.status.last_result = PROBE_CALIB_RESULT_OK;
}

void probe_calib_get(probe_calib_record *out)
{
    if (out) *out = calib.current;
}

static int flash_program_record(unsigned slot, const probe_calib_record *r)
{
    const uint32_t *word = (const uint32_t *)r;
    uint32_t address = CALIB_SECTOR_BASE + slot * CALIB_SECTOR_SIZE;
    unsigned i;
    if (flash_sector_erase(address) != FLASH_OPERATE_DONE) return 0;
    for (i = 0; i < sizeof(*r) / sizeof(uint32_t); ++i) {
        if (flash_word_program(address + i * 4U, word[i]) != FLASH_OPERATE_DONE) return 0;
    }
    return memcmp((const void *)address, r, sizeof(*r)) == 0;
}

int probe_calib_save(const probe_calib_record *record)
{
    probe_calib_record copy, a, b;
    int va, vb;
    unsigned slot;
    if (!record || record->magic != PROBE_CALIB_MAGIC || record->version != PROBE_CALIB_VERSION ||
        !record->valid_flags || (record->valid_flags & ~(PROBE_CALIB_VALID_ACCEL |
        PROBE_CALIB_VALID_GYRO | PROBE_CALIB_VALID_ICM_ACCEL))) return 0;
    copy = *record;
    copy.generation = 1;
    va = load_slot(0, &a); vb = load_slot(1, &b);
    /* Write the older slot (or slot 1 when only slot 0 is in use) so a power
     * loss during programming leaves the previous record intact. */
    if (va && !vb)
        slot = 1;
    else if (!va) slot = 0;
    else if (!vb) slot = 1;
    else slot = newer(&a, &b) ? 1 : 0;
    if (va || vb) {
        const probe_calib_record *latest = !vb || (va && newer(&a, &b)) ? &a : &b;
        copy.generation = latest->generation + 1U;
    }
    copy.crc = probe_frame_crc32c((const uint8_t *)&copy, CALIB_RECORD_PAYLOAD);
    flash_unlock();
    {
        int ok = flash_program_record(slot, &copy);
        flash_lock();
        if (!ok) return 0;
    }
    calib.current = copy;
    return 1;
}

static int fit_accel(const double means[6][3], double nominal_counts_per_g,
                     float bias_g[3], float scale[3])
{
    /* Per-axis algebraic solution from +-1 g pairs:
     * scale_raw = (p - n) / 2 counts per g; bias = (p + n) / 2 counts.
     * Orientation order: +X, -X, +Y, -Y, +Z, -Z. */
    unsigned i;
    double g[3];
    if (!means || !bias_g || !scale || nominal_counts_per_g <= 0) return 0;
    for (i = 0; i < 3; ++i) {
        double p = means[2 * i][i];
        double n = means[2 * i + 1][i];
        double counts_per_g = (p - n) / 2.0;
        double bias = (p + n) / 2.0;
        if (!(counts_per_g >= nominal_counts_per_g * 0.5 &&
              counts_per_g <= nominal_counts_per_g * 1.5))
            return 0; /* orientation mix-up or wrong range */
        bias_g[i] = (float)(bias / nominal_counts_per_g);
        scale[i] = (float)(nominal_counts_per_g / counts_per_g);
        g[i] = counts_per_g;
    }
    /* Gravity magnitude gate: fitted counts-per-g must reproduce 1 g on all
     * axes within 5 percent of each other. */
    for (i = 0; i < 3; ++i) {
        double ratio = g[i] / g[(i + 1) % 3];
        if (ratio > 1.05 || ratio < 0.95) return 0;
    }
    return 1;
}

int probe_calib_fit_accel_six_positions(const double means[6][3],
                                        probe_calib_record *out)
{
    if (!out) return 0;
    defaults(out);
    if (!fit_accel(means, 52428.8, out->accel_bias_g, out->accel_scale)) return 0;
    out->valid_flags |= PROBE_CALIB_VALID_ACCEL;
    return 1;
}

int probe_calib_fit_icm_accel_six_positions(const double means[6][3],
                                            double nominal_counts_per_g,
                                            probe_calib_record *out)
{
    if (!out) return 0;
    defaults(out);
    if (!fit_accel(means, nominal_counts_per_g,
                   out->icm_accel_bias_g, out->icm_accel_scale)) return 0;
    out->valid_flags |= PROBE_CALIB_VALID_ICM_ACCEL;
    return 1;
}

void probe_calib_fit_gyro_static(const double mean_dps[3], probe_calib_record *out)
{
    unsigned i;
    defaults(out);
    for (i = 0; i < 3; ++i) out->gyro_bias_dps[i] = (float)mean_dps[i];
    out->valid_flags |= PROBE_CALIB_VALID_GYRO;
}

static void finish_capture(void)
{
    double mean[6];
    unsigned i, channels = calib.status.target == PROBE_CALIB_TARGET_ICM_GYRO ? 6U : 3U;
    for (i = 0; i < channels; ++i) {
        double variance;
        double limit;
        if (calib.status.target == PROBE_CALIB_TARGET_ICM_GYRO)
            limit = i < 3 ? calib.gyro_counts_per_dps * 0.625 :
                            calib.accel_counts_per_g * 0.03;
        else
            limit = calib.accel_counts_per_g * 0.03;
        mean[i] = calib.sum[i] / calib.status.sample_count;
        variance = calib.square[i] / calib.status.sample_count - mean[i] * mean[i];
        if (!(variance <= limit * limit)) goto rejected;
    }
    if (calib.status.target == PROBE_CALIB_TARGET_ICM_GYRO) {
        double gravity = 0;
        for (i = 0; i < 3; ++i) {
            if (fabs(mean[i]) > calib.gyro_counts_per_dps * 5.0) goto rejected;
            gravity += mean[i + 3] * mean[i + 3];
            calib.candidate.gyro_bias_dps[i] = (float)(mean[i] / calib.gyro_counts_per_dps);
        }
        if (gravity < 0.85 * 0.85 * calib.accel_counts_per_g * calib.accel_counts_per_g ||
            gravity > 1.15 * 1.15 * calib.accel_counts_per_g * calib.accel_counts_per_g)
            goto rejected;
        calib.candidate.temp_c = (float)(25.0 + calib.temp_sum /
                                         calib.status.sample_count / 2.07);
        calib.candidate.valid_flags |= PROBE_CALIB_VALID_GYRO;
        calib.status.dirty = 1;
    } else {
        unsigned set = calib.status.target == PROBE_CALIB_TARGET_ADXL_ACCEL ? 0U : 1U;
        unsigned axis = calib.status.face / 2U;
        double expected = calib.status.face & 1U ? -calib.accel_counts_per_g :
                                                  calib.accel_counts_per_g;
        for (i = 0; i < 3; ++i)
            if (fabs(mean[i] - (i == axis ? expected : 0.0)) >
                calib.accel_counts_per_g * 0.15) goto rejected;
        memcpy(calib.faces[set][calib.status.face], mean, 3U * sizeof(double));
        calib.face_mask[set] |= (uint8_t)(1U << calib.status.face);
        calib.status.completed_mask = calib.face_mask[set];
        if (calib.face_mask[set] == 0x3FU) {
            float bias[3], scale[3];
            if (!fit_accel((const double (*)[3])calib.faces[set],
                           calib.accel_counts_per_g, bias, scale)) goto rejected;
            if (set == 0U) {
                memcpy(calib.candidate.accel_bias_g, bias, sizeof(bias));
                memcpy(calib.candidate.accel_scale, scale, sizeof(scale));
                calib.candidate.valid_flags |= PROBE_CALIB_VALID_ACCEL;
            } else {
                memcpy(calib.candidate.icm_accel_bias_g, bias, sizeof(bias));
                memcpy(calib.candidate.icm_accel_scale, scale, sizeof(scale));
                calib.candidate.valid_flags |= PROBE_CALIB_VALID_ICM_ACCEL;
            }
            calib.status.dirty = 1;
        }
    }
    calib.status.state = PROBE_CALIB_IDLE;
    calib.status.last_result = PROBE_CALIB_RESULT_OK;
    calib.status.candidate_valid_flags = calib.candidate.valid_flags;
    return;
rejected:
    calib.status.state = PROBE_CALIB_IDLE;
    calib.status.last_result = PROBE_CALIB_RESULT_REJECTED;
}

uint8_t probe_calib_begin(uint8_t target, uint8_t face,
                          const probe_acquisition_config *config, uint64_t now_us)
{
    unsigned set;
    if (calib.status.state == PROBE_CALIB_CAPTURING) return PROBE_CALIB_RESULT_BUSY;
    if (!config || target < PROBE_CALIB_TARGET_ADXL_ACCEL ||
        target > PROBE_CALIB_TARGET_ICM_GYRO ||
        (target != PROBE_CALIB_TARGET_ICM_GYRO && face >= 6U) ||
        (target == PROBE_CALIB_TARGET_ICM_GYRO && face != 0U))
        return PROBE_CALIB_RESULT_INVALID;
    memset(calib.sum, 0, sizeof(calib.sum));
    memset(calib.square, 0, sizeof(calib.square));
    calib.temp_sum = 0;
    calib.status.state = PROBE_CALIB_CAPTURING;
    calib.status.target = target;
    calib.status.face = face;
    calib.status.sample_count = 0;
    calib.status.last_result = PROBE_CALIB_RESULT_IN_PROGRESS;
    calib.sequence_valid = 0;
    calib.deadline_us = now_us + 3000000ULL;
    if (target == PROBE_CALIB_TARGET_ADXL_ACCEL) {
        calib.status.target_samples = config->adxl.odr_hz;
        calib.accel_counts_per_g = 524288.0 / config->adxl.range_g;
        set = 0;
    } else {
        calib.status.target_samples = config->icm.odr_hz;
        calib.accel_counts_per_g = 32768.0 / config->icm.accel_g;
        calib.gyro_counts_per_dps = 32768.0 / config->icm.gyro_dps;
        set = 1;
    }
    if (!calib.status.target_samples || !calib.accel_counts_per_g ||
        (target == PROBE_CALIB_TARGET_ICM_GYRO && !calib.gyro_counts_per_dps)) {
        calib.status.state = PROBE_CALIB_IDLE;
        calib.status.last_result = PROBE_CALIB_RESULT_INVALID;
        return PROBE_CALIB_RESULT_INVALID;
    }
    calib.status.completed_mask = target == PROBE_CALIB_TARGET_ICM_GYRO ? 0 :
                                  calib.face_mask[set];
    return PROBE_CALIB_RESULT_IN_PROGRESS;
}

void probe_calib_cancel(void)
{
    if (calib.status.state == PROBE_CALIB_CAPTURING) {
        calib.status.state = PROBE_CALIB_IDLE;
        calib.status.last_result = PROBE_CALIB_RESULT_OK;
    }
}

void probe_calib_reset_progress(uint8_t target)
{
    unsigned set;
    if (calib.status.state == PROBE_CALIB_CAPTURING) return;
    if (target != PROBE_CALIB_TARGET_ADXL_ACCEL &&
        target != PROBE_CALIB_TARGET_ICM_ACCEL) return;
    set = target == PROBE_CALIB_TARGET_ADXL_ACCEL ? 0U : 1U;
    calib.face_mask[set] = 0;
    memset(calib.faces[set], 0, sizeof(calib.faces[set]));
    calib.status.target = target;
    calib.status.completed_mask = 0;
    calib.status.last_result = PROBE_CALIB_RESULT_OK;
}

void probe_calib_ingest(probe_spi_bus bus, const probe_raw_batch *batch, uint64_t now_us)
{
    unsigned offset, axis, width;
    int wanted_bus;
    if (calib.status.state != PROBE_CALIB_CAPTURING || !batch) return;
    wanted_bus = calib.status.target == PROBE_CALIB_TARGET_ADXL_ACCEL ?
                 PROBE_SPI_ADXL : PROBE_SPI_ICM;
    if ((int)bus != wanted_bus) return;
    if ((batch->flags & PROBE_BATCH_NEAR_FULL_SCALE) ||
        (calib.sequence_valid && (batch->segment != calib.capture_segment ||
         batch->first_sequence != calib.next_sequence))) goto rejected;
    calib.capture_segment = batch->segment;
    calib.next_sequence = batch->first_sequence + batch->samples;
    calib.sequence_valid = 1;
    width = bus == PROBE_SPI_ADXL ? ADXL357_FIFO_XYZ_BYTES : ICM42688_FIFO_PACKET_BYTES;
    for (offset = 0; offset + width <= batch->size &&
         calib.status.sample_count < calib.status.target_samples; offset += width) {
        int32_t values[6];
        if (bus == PROBE_SPI_ADXL) {
            if (!adxl357_decode_fifo_xyz(batch->data + offset, width, values)) goto rejected;
        } else {
            icm42688_fifo_sample sample;
            if (!icm42688_decode_fifo(batch->data + offset, width, &sample)) goto rejected;
            if (calib.status.target == PROBE_CALIB_TARGET_ICM_GYRO) {
                for (axis = 0; axis < 3; ++axis) {
                    values[axis] = sample.gyro[axis];
                    values[axis + 3] = sample.accel[axis];
                }
                calib.temp_sum += sample.temperature;
            } else {
                for (axis = 0; axis < 3; ++axis) values[axis] = sample.accel[axis];
            }
        }
        for (axis = 0; axis < (calib.status.target == PROBE_CALIB_TARGET_ICM_GYRO ? 6U : 3U); ++axis) {
            calib.sum[axis] += values[axis];
            calib.square[axis] += (double)values[axis] * values[axis];
        }
        ++calib.status.sample_count;
    }
    if (calib.status.sample_count == calib.status.target_samples) finish_capture();
    else if (now_us >= calib.deadline_us) goto rejected;
    return;
rejected:
    calib.status.state = PROBE_CALIB_IDLE;
    calib.status.last_result = PROBE_CALIB_RESULT_REJECTED;
}

void probe_calib_poll(uint64_t now_us)
{
    if (calib.status.state == PROBE_CALIB_CAPTURING && now_us >= calib.deadline_us) {
        calib.status.state = PROBE_CALIB_IDLE;
        calib.status.last_result = PROBE_CALIB_RESULT_REJECTED;
    }
}

void probe_calib_get_status(probe_calib_status *out)
{
    if (!out) return;
    calib.status.candidate_valid_flags = calib.candidate.valid_flags;
    *out = calib.status;
}

void probe_calib_get_candidate(probe_calib_record *out)
{
    if (out) *out = calib.candidate;
}

uint8_t probe_calib_save_candidate(uint32_t created_ms)
{
    if (calib.status.state == PROBE_CALIB_CAPTURING) return PROBE_CALIB_RESULT_BUSY;
    if (!calib.status.dirty || !calib.candidate.valid_flags) return PROBE_CALIB_RESULT_NOT_READY;
    calib.candidate.created_ms = created_ms;
    if (!probe_calib_save(&calib.candidate)) return PROBE_CALIB_RESULT_INTERNAL;
    calib.candidate = calib.current;
    calib.status.dirty = 0;
    calib.status.candidate_valid_flags = calib.candidate.valid_flags;
    return PROBE_CALIB_RESULT_OK;
}
