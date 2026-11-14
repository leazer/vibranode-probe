/**
 * @file test_probe_agm.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-06
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <stdint.h>
#include <string.h>
#include "invn_algo_agm.h"

/* Use the supplied vector unchanged, including its original full scales.
 * These test-only scales do not expand the product configuration range. */
static const int32_t samples[][24] = {
#include "out_invn_agm_1.h"
};

int test_main(void)
{
    InvnAlgoAGMConfig config;
    InvnAlgoAGMInput input;
    InvnAlgoAGMOutput output;
    unsigned row, axis;
    memset(&config, 0, sizeof(config));
    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));
    config.acc_fsr = 32;
    config.gyr_fsr = 4000;
    config.acc_odr_us = 10000;
    config.gyr_odr_us = 10000;
    config.mag_odr_us = 10000;
    config.mag_sc_q16 = 9830;
    config.temp_offset = 25 * 65536;
    config.temp_sensitivity = (int32_t)((100LL << 30) / 13248);
    if (invn_algo_agm_init(&config)) return -1;
    for (row = 0; row < sizeof(samples) / sizeof(samples[0]); ++row) {
        input.mask = 7;
        input.sRimu_time_us = samples[row][0];
        input.sRmag_time_us = samples[row][0];
        input.sRtemp_data = (int16_t)samples[row][7];
        for (axis = 0; axis < 3; ++axis) {
            input.sRacc_data[axis] = samples[row][1 + axis];
            input.sRgyr_data[axis] = samples[row][4 + axis];
            input.sRmag_data[axis] = samples[row][8 + axis];
        }
        invn_algo_agm_process(&input, &output);
        for (axis = 0; axis < 4; ++axis)
            if (output.rv_quat_q30[axis] != samples[row][11 + axis])
                return (int)(row + 1);
        for (axis = 0; axis < 3; ++axis)
            if (output.acc_bias_q16[axis] != samples[row][15 + axis] ||
                output.gyr_bias_q16[axis] != samples[row][18 + axis] ||
                output.mag_bias_q16[axis] != samples[row][21 + axis])
                return (int)(row + 1);
    }
    /* Product-default six-axis smoke check, separate from the official vector. */
    memset(&config, 0, sizeof(config));
    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));
    config.acc_fsr = 16;
    config.gyr_fsr = 500;
    config.acc_odr_us = 5000;
    config.gyr_odr_us = 5000;
    config.temp_offset = 25 * 65536;
    config.temp_sensitivity = (int32_t)((100LL << 30) / 207);
    if (invn_algo_agm_init(&config)) return -2;
    input.mask = INVN_ALGO_AGM_INPUT_MASK_ACC | INVN_ALGO_AGM_INPUT_MASK_GYR;
    input.sRacc_data[2] = 32768; /* 1 g at +/-16 g in the library's input scale */
    for (row = 1; row <= 1000; ++row) {
        input.sRimu_time_us = (int64_t)row * 5000;
        invn_algo_agm_process(&input, &output);
    }
    if (!(output.mask & INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AG)) return -3;
    {
        double norm_squared = 0;
        for (axis = 0; axis < 4; ++axis) {
            double value = (double)output.grv_quat_q30[axis] / 1073741824.0;
            norm_squared += value * value;
        }
        if (!(norm_squared > 0.99 && norm_squared < 1.01)) return -4;
    }
    return 0;
}
