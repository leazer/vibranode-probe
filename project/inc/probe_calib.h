/**
 * @file probe_calib.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_CALIB_H
#define PROBE_CALIB_H
#include <stdint.h>
#include "probe_acquisition.h"

/* M8 calibration records and dual-record flash storage (probe design
 * section 7). The probe stores and serves calibration; applying it to
 * samples happens on the analysis side, raw streams stay untouched.
 * Stored in the last two 2 KB sectors of the 256 KB flash, one record per
 * sector, newest valid wins. */

#define PROBE_CALIB_MAGIC 0x314E4256U /* "VBN1" little-endian */
#define PROBE_CALIB_VERSION 3U

enum {
    PROBE_CALIB_VALID_ACCEL = 0x01,
    PROBE_CALIB_VALID_GYRO = 0x02,
    PROBE_CALIB_VALID_ICM_ACCEL = 0x04
};

typedef struct {
    uint32_t magic;
    uint8_t version;
    uint8_t valid_flags;
    uint16_t reserved;
    float accel_bias_g[3]; /* corrected_g = (raw / nominal_counts_per_g - bias_g) * scale */
    float accel_scale[3]; /* 1.0 = nominal */
    float icm_accel_bias_g[3];
    float icm_accel_scale[3];
    float gyro_bias_dps[3];
    float temp_c;
    uint32_t created_ms;
    uint32_t generation; /* storage-owned save counter; independent of boot uptime */
    uint32_t crc; /* CRC-32C over all preceding bytes */
} probe_calib_record;

/* Load the newest valid record at boot (defaults: scale 1, bias 0). */
void probe_calib_init(void);
void probe_calib_get(probe_calib_record *out);
/* Persist a record; returns 0 on failure (flash error or bad content). */
int probe_calib_save(const probe_calib_record *record);

/* Six-position fit: means are average raw counts (ADXL, 20-bit) for the
 * orientations +X, -X, +Y, -Y, +Z, -Z facing up. Fills bias/scale for all
 * three axes and sets VALID_ACCEL; returns 0 when the magnitude gate fails
 * (bad orientation data or residual gravity error above 5 percent). */
int probe_calib_fit_accel_six_positions(const double means[6][3],
                                        probe_calib_record *out);
int probe_calib_fit_icm_accel_six_positions(const double means[6][3],
                                            double nominal_counts_per_g,
                                            probe_calib_record *out);
/* Gyro static bias: mean counts at rest; ±500 dps default scale is applied
 * by the caller converting to dps beforehand. Marks VALID_GYRO. */
void probe_calib_fit_gyro_static(const double mean_dps[3], probe_calib_record *out);

enum {
    PROBE_CALIB_TARGET_ADXL_ACCEL = 1,
    PROBE_CALIB_TARGET_ICM_ACCEL = 2,
    PROBE_CALIB_TARGET_ICM_GYRO = 3
};
enum { PROBE_CALIB_IDLE, PROBE_CALIB_CAPTURING };
enum {
    PROBE_CALIB_RESULT_OK = 0,
    PROBE_CALIB_RESULT_BUSY = 1,
    PROBE_CALIB_RESULT_INVALID = 4,
    PROBE_CALIB_RESULT_NOT_READY = 6,
    PROBE_CALIB_RESULT_IN_PROGRESS = 7,
    PROBE_CALIB_RESULT_INTERNAL = 9,
    PROBE_CALIB_RESULT_REJECTED = 10
};
typedef struct {
    uint8_t state, target, face, completed_mask;
    uint8_t candidate_valid_flags, dirty, last_result, reserved;
    uint32_t sample_count, target_samples;
} probe_calib_status;

/* Nonblocking production calibration session. One capture records one face
 * (0..5 = +X,-X,+Y,-Y,+Z,-Z) or one ICM gyro-static window. */
uint8_t probe_calib_begin(uint8_t target, uint8_t face,
                          const probe_acquisition_config *config, uint64_t now_us);
void probe_calib_cancel(void);
void probe_calib_reset_progress(uint8_t target);
void probe_calib_ingest(probe_spi_bus bus, const probe_raw_batch *batch, uint64_t now_us);
void probe_calib_poll(uint64_t now_us);
void probe_calib_get_status(probe_calib_status *out);
void probe_calib_get_candidate(probe_calib_record *out);
uint8_t probe_calib_save_candidate(uint32_t created_ms);

#endif
