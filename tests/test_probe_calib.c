/**
 * @file test_probe_calib.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <math.h>
#include <string.h>
#include "at32f413_wk_config.h"
#include "at32f413_flash.h"
#include "probe_calib.h"
#include "probe_frame.h"

#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

/* Fake flash: two 2 KB sectors backing the record area, mapped into the
 * emulated CPU address space by the runner. */
#define SECTOR_SIZE 2048U
#define CAL_BASE 0x0803F000U
static int writes_left = -1;

typedef struct {
    uint32_t magic;
    uint8_t version, valid_flags;
    uint16_t reserved;
    float accel_bias_g[3], accel_scale[3], gyro_bias_dps[3], temp_c;
    uint32_t created_ms, generation, crc;
} legacy_v2_record;

flash_status_type flash_sector_erase(uint32_t address)
{
    unsigned slot = (address - CAL_BASE) / SECTOR_SIZE;
    if (slot > 1) return FLASH_PROGRAM_ERROR;
    memset((void *)(CAL_BASE + slot * SECTOR_SIZE), 0xFF, SECTOR_SIZE);
    return FLASH_OPERATE_DONE;
}
flash_status_type flash_word_program(uint32_t address, uint32_t data)
{
    if (writes_left == 0) return FLASH_PROGRAM_ERROR;
    if (writes_left > 0) --writes_left;
    if (address < CAL_BASE || address >= CAL_BASE + 2 * SECTOR_SIZE) return FLASH_PROGRAM_ERROR;
    memcpy((void *)address, &data, 4);
    return FLASH_OPERATE_DONE;
}
void flash_unlock(void) { }
void flash_lock(void) { }

static double means[6][3];

static void fill_orientation_pair(unsigned axis, double bias_counts,
                                  double counts_per_g)
{
    unsigned i;
    for (i = 0; i < 3; ++i) {
        means[2 * axis][i] = bias_counts;
        means[2 * axis + 1][i] = bias_counts;
    }
    means[2 * axis][axis] = bias_counts + counts_per_g;
    means[2 * axis + 1][axis] = bias_counts - counts_per_g;
}

static void fill_icm_batch(probe_raw_batch *batch, unsigned face, uint64_t sequence)
{
    unsigned sample, axis;
    memset(batch, 0, sizeof(*batch));
    batch->segment = 1;
    batch->first_sequence = sequence;
    batch->samples = 32;
    batch->size = 32 * ICM42688_FIFO_PACKET_BYTES;
    for (sample = 0; sample < 32; ++sample) {
        uint8_t *p = batch->data + sample * ICM42688_FIFO_PACKET_BYTES;
        p[0] = 0x68;
        for (axis = 0; axis < 3; ++axis) {
            int16_t value = axis == face / 2 ? (face & 1 ? -2048 : 2048) : 0;
            p[1 + axis * 2] = (uint8_t)((uint16_t)value >> 8);
            p[2 + axis * 2] = (uint8_t)value;
        }
    }
}

int test_main(void)
{
    probe_calib_record rec;
    unsigned axis;

    /* Existing v2 records migrate in RAM so field expansion does not erase
     * deployed ADXL/gyro calibration. The next save writes v3. */
    {
        legacy_v2_record old;
        memset((void *)CAL_BASE, 0xFF, 2 * SECTOR_SIZE);
        memset(&old, 0, sizeof(old));
        old.magic = PROBE_CALIB_MAGIC; old.version = 2;
        old.valid_flags = PROBE_CALIB_VALID_ACCEL | PROBE_CALIB_VALID_GYRO;
        old.accel_scale[0] = old.accel_scale[1] = old.accel_scale[2] = 1.0f;
        old.accel_bias_g[0] = 0.125f; old.gyro_bias_dps[1] = -0.5f;
        old.created_ms = 88; old.generation = 9;
        old.crc = probe_frame_crc32c((const uint8_t *)&old, sizeof(old) - 4U);
        memcpy((void *)CAL_BASE, &old, sizeof(old));
        probe_calib_init(); probe_calib_get(&rec);
        CHECK(rec.version == PROBE_CALIB_VERSION && rec.generation == 9);
        CHECK(rec.accel_bias_g[0] == 0.125f && rec.gyro_bias_dps[1] == -0.5f);
        CHECK(rec.icm_accel_scale[0] == 1.0f);
    }

    /* Empty flash: defaults (scale 1, bias 0, nothing valid). */
    memset((void *)CAL_BASE, 0xFF, 2 * SECTOR_SIZE);
    probe_calib_init();
    probe_calib_get(&rec);
    CHECK(rec.magic == PROBE_CALIB_MAGIC);
    CHECK(rec.valid_flags == 0);
    CHECK(rec.accel_scale[0] == 1.0f && rec.accel_bias_g[0] == 0.0f);
    CHECK(rec.icm_accel_scale[0] == 1.0f && rec.icm_accel_bias_g[0] == 0.0f);

    /* Six-position fit recovers injected bias and scale. */
    {
        double bias[3] = {-2000.0, 3000.0, 1500.0};
        double scale[3] = {1.02, 0.98, 1.01};
        for (axis = 0; axis < 3; ++axis)
            fill_orientation_pair(axis, bias[axis], 52428.8 * scale[axis]);
        CHECK(probe_calib_fit_accel_six_positions((const double (*)[3])means, &rec) == 1);
        CHECK(rec.valid_flags & PROBE_CALIB_VALID_ACCEL);
        for (axis = 0; axis < 3; ++axis) {
            double got_bias = (double)rec.accel_bias_g[axis];
            double want_bias = bias[axis] / 52428.8;
            CHECK(got_bias > want_bias - 0.001 && got_bias < want_bias + 0.001);
            {
                /* Stored scale is the correction applied to (raw - bias):
                 * corrected g = (raw - bias) * scale / lsbs_per_g. */
                double want_scale = 1.0 / scale[axis];
                CHECK(rec.accel_scale[axis] > want_scale - 0.001 &&
                      rec.accel_scale[axis] < want_scale + 0.001);
            }
        }
    }

    /* Magnitude gate: a mixed-up orientation (counts per g 20% off) fails. */
    for (axis = 0; axis < 3; ++axis)
        fill_orientation_pair(axis, 0.0, 52428.8);
    means[4][2] += 52428.8 * 0.2; /* Z pair unbalanced */
    CHECK(probe_calib_fit_accel_six_positions((const double (*)[3])means, &rec) == 0);

    /* ICM six-face calibration has independent coefficients and validity. */
    for (axis = 0; axis < 3; ++axis)
        fill_orientation_pair(axis, 20.0 * (axis + 1), 2048.0 * (1.0 + 0.01 * axis));
    CHECK(probe_calib_fit_icm_accel_six_positions((const double (*)[3])means, 2048.0, &rec) == 1);
    CHECK(rec.valid_flags & PROBE_CALIB_VALID_ICM_ACCEL);
    CHECK(rec.icm_accel_bias_g[0] > 0.009f && rec.icm_accel_bias_g[0] < 0.011f);
    CHECK(rec.icm_accel_scale[2] > 0.979f && rec.icm_accel_scale[2] < 0.981f);

    /* Save persists and survives a reboot; second save rotates sectors. */
    for (axis = 0; axis < 3; ++axis)
        fill_orientation_pair(axis, 0.0, 52428.8);
    CHECK(probe_calib_fit_accel_six_positions((const double (*)[3])means, &rec) == 1);
    rec.created_ms = 100;
    CHECK(probe_calib_save(&rec) == 1);
    probe_calib_init(); /* simulated reboot */
    probe_calib_get(&rec);
    CHECK((rec.valid_flags & PROBE_CALIB_VALID_ACCEL) && rec.created_ms == 100);

    probe_calib_fit_gyro_static((const double[3]){0.5, -1.2, 0.3}, &rec);
    rec.created_ms = 200;
    CHECK(probe_calib_save(&rec) == 1);
    probe_calib_init();
    probe_calib_get(&rec);
    CHECK(rec.created_ms == 200);
    CHECK(rec.gyro_bias_dps[0] > 0.4f && rec.gyro_bias_dps[0] < 0.6f);
    CHECK((rec.valid_flags & PROBE_CALIB_VALID_GYRO) != 0);
    /* Accel fields survive inside the merged newest record. */
    CHECK((rec.valid_flags & PROBE_CALIB_VALID_ACCEL) == 0); /* gyro-only record */

    /* A reboot resets uptime: the latest save must still win at boot. */
    rec.created_ms = 1;
    CHECK(probe_calib_save(&rec) == 1);
    probe_calib_init();
    probe_calib_get(&rec);
    CHECK(rec.created_ms == 1);

    /* A corrupted newest record falls back to the older one. */
    {
        uint8_t *raw = (uint8_t *)CAL_BASE;
        raw[10] ^= 0xFF; /* break newest slot 0 */
        probe_calib_init();
        probe_calib_get(&rec);
        CHECK(rec.created_ms == 200); /* previous slot 1 remains valid */
    }
    /* Every possible programming interruption retains the previous record. */
    for (axis = 0; axis < sizeof(rec) / 4; ++axis) {
        rec.created_ms = 999;
        writes_left = (int)axis;
        CHECK(probe_calib_save(&rec) == 0);
        probe_calib_init(); probe_calib_get(&rec);
        CHECK(rec.created_ms == 200);
    }
    /* Nonblocking ICM six-face capture accepts one second per face and
     * merges an independent candidate without writing flash. */
    {
        probe_acquisition_config config = {{250, 10, 0}, {200, 16, 500}};
        probe_raw_batch batch;
        probe_calib_status status;
        unsigned face, part;
        writes_left = -1;
        probe_calib_init();
        probe_calib_reset_progress(PROBE_CALIB_TARGET_ICM_ACCEL);
        for (face = 0; face < 6; ++face) {
            CHECK(probe_calib_begin(PROBE_CALIB_TARGET_ICM_ACCEL, (uint8_t)face,
                                    &config, face * 2000000ULL) == PROBE_CALIB_RESULT_IN_PROGRESS);
            for (part = 0; part < 7; ++part) {
                fill_icm_batch(&batch, face, part * 32U);
                probe_calib_ingest(PROBE_SPI_ICM, &batch,
                                   face * 2000000ULL + part * 100000ULL);
            }
            probe_calib_get_status(&status);
            CHECK(status.state == PROBE_CALIB_IDLE && status.last_result == PROBE_CALIB_RESULT_OK);
            CHECK(status.completed_mask == (uint8_t)((1U << (face + 1U)) - 1U));
        }
        probe_calib_get_candidate(&rec);
        CHECK(rec.valid_flags & PROBE_CALIB_VALID_ICM_ACCEL);
        CHECK(rec.icm_accel_scale[0] > 0.999f && rec.icm_accel_scale[0] < 1.001f);
        probe_calib_get_status(&status);
        CHECK(status.dirty && status.sample_count == 200);
    }
    return 0;
}
