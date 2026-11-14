/**
 * @file adxl357.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "adxl357.h"
#include "adxl357_regs.h"

/* Preserve the board-validated M2 profile. Reset is self-clearing.
 * Interrupts signal data ready here, not FIFO watermark. */
static const sensor_register_setting diagnostic_settings[] = {
    {ADXL357_REG_RESET, 0x52, 0x00, 20000},
    {ADXL357_REG_POWER_CTL, 0x01, 0x07, 0},
    {ADXL357_REG_FILTER, 0x00, 0x7F, 0}, /* 4 kSPS / 1 kHz, HPF off */
    {ADXL357_REG_RANGE, 0xC1, 0xC3, 0}, /* +/-10 g, active-high INT */
    {ADXL357_REG_INT_MAP, 0x01, 0xFF, 0}, /* DATA_RDY on INT1 */
    {ADXL357_REG_POWER_CTL, 0x00, 0x07, 50000}
};

const sensor_register_setting *adxl357_diagnostic_settings(unsigned *count)
{
    if (count) *count = sizeof(diagnostic_settings) / sizeof(diagnostic_settings[0]);
    return diagnostic_settings;
}

uint8_t adxl357_spi_command(uint8_t reg, int read)
{
    return (uint8_t)((reg << 1) | (read ? 1U : 0U));
}

int adxl357_identity_valid(const uint8_t *id, unsigned size)
{
    return id && size >= 4 && id[0] == 0xAD && id[1] == 0x1D && id[2] == 0xED;
}

int adxl357_decode_snapshot(const uint8_t *data, unsigned size,
                           int32_t accel[3])
{
    unsigned axis;
    if (!data || !accel || size < ADXL357_SNAPSHOT_BYTES) return 0;
    for (axis = 0; axis < 3; ++axis) {
        const uint8_t *p = data + 4 + 3 * axis;
        uint32_t value = ((uint32_t)p[0] << 12) | ((uint32_t)p[1] << 4) | (p[2] >> 4);
        accel[axis] = value & 0x80000U ? (int32_t)value - 1048576 : (int32_t)value;
    }
    return 1;
}

unsigned adxl357_build_fifo_config(const adxl357_config *config,
                                 sensor_register_setting *steps, unsigned capacity)
{
    static const sensor_register_setting defaults[] = {
        {ADXL357_REG_RESET, 0x52, 0, 20000},
        {ADXL357_REG_POWER_CTL, 1, 7, 0},
        {ADXL357_REG_FILTER, 0, 0x7F, 0},
        {ADXL357_REG_RANGE, 0xC1, 0xC3, 0},
        {0x29, 24, 0x7F, 0}, /* FIFO watermark: axis entries */
        {ADXL357_REG_INT_MAP, 6, 0xFF, 0}, /* watermark and overflow on INT1 */
        {ADXL357_REG_POWER_CTL, 0, 7, 0}
    };
    unsigned odr, range, i, groups;
    if (!config || !steps || capacity < 7 || config->hpf > 6) return 0;
    switch (config->odr_hz) {
    case 4000: odr = 0; break;
    case 2000: odr = 1; break;
    case 1000: odr = 2; break;
    case 500: odr = 3; break;
    case 250: odr = 4; break;
    default: return 0;
    }
    switch (config->range_g) {
    case 10: range = 1; break;
    case 20: range = 2; break;
    case 40: range = 3; break;
    default: return 0;
    }
    for (i = 0; i < 7; ++i) steps[i] = defaults[i];
    groups = config->odr_hz / 500;
    if (!groups) groups = 1;
    steps[2].value = (uint8_t)(odr | (config->hpf << 4));
    steps[3].value = (uint8_t)(0xC0 | range);
    steps[4].value = (uint8_t)(groups * 3);
    return 7;
}

int adxl357_decode_fifo_xyz(const uint8_t *data, unsigned size, int32_t accel[3])
{
    unsigned axis;
    if (!data || !accel || size < ADXL357_FIFO_XYZ_BYTES) return 0;
    /* Reject the entire XYZ group before touching output. */
    for (axis = 0; axis < 3; ++axis) {
        uint8_t marker = data[3 * axis + 2] & 3;
        if (marker != (axis == 0 ? 1 : 0)) return 0;
    }
    for (axis = 0; axis < 3; ++axis) {
        const uint8_t *p = data + 3 * axis;
        uint32_t value = ((uint32_t)p[0] << 12) | ((uint32_t)p[1] << 4) | (p[2] >> 4);
        accel[axis] = value & 0x80000U ? (int32_t)value - 1048576 : (int32_t)value;
    }
    return 1;
}
