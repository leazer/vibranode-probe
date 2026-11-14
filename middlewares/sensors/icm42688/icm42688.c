/**
 * @file icm42688.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "icm42688.h"
#define ICM42688P
#include "Icm426xxDefs.h"

/* Preserve the board-validated M2 profile. Reset is self-clearing.
 * Interrupts signal data ready here, not FIFO watermark. */
static const sensor_register_setting diagnostic_settings[] = {
    {MPUREG_DEVICE_CONFIG, ICM426XX_DEVICE_CONFIG_RESET_EN, 0x00, 20000},
    {MPUREG_REG_BANK_SEL, 0x00, 0x07, 0},
    {MPUREG_GYRO_CONFIG0, ICM426XX_GYRO_CONFIG0_FS_SEL_500dps | ICM426XX_GYRO_CONFIG0_ODR_8_KHZ, 0xFF, 0}, /* +/-500 dps, 8 kSPS */
    {MPUREG_ACCEL_CONFIG0, ICM426XX_ACCEL_CONFIG0_FS_SEL_16g | ICM426XX_ACCEL_CONFIG0_ODR_8_KHZ, 0xFF, 0}, /* +/-16 g, 8 kSPS */
    {MPUREG_INT_CONFIG, ICM426XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT_CONFIG_INT1_POLARITY_HIGH, 0x07, 0}, /* INT1 active-high push-pull, pulse */
    {MPUREG_INT_CONFIG1, 0x60, 0x70, 0}, /* 8 us pulse; deassert guard disabled; INT_ASYNC_RESET=0 */
    {MPUREG_INT_SOURCE0, BIT_INT_SOURCE0_UI_DRDY_INT1_EN, 0x7F, 0}, /* UI data ready */
    {MPUREG_PWR_MGMT_0, ICM426XX_PWR_MGMT_0_GYRO_MODE_LN | ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN, 0x3F, 50000}
};

const sensor_register_setting *icm42688_diagnostic_settings(unsigned *count)
{
    if (count) *count = sizeof(diagnostic_settings) / sizeof(diagnostic_settings[0]);
    return diagnostic_settings;
}

uint8_t icm42688_spi_command(uint8_t reg, int read)
{
    return (uint8_t)(reg | (read ? 0x80U : 0U));
}

int icm42688_identity_valid(const uint8_t *id, unsigned size)
{
    return id && size >= 1 && id[0] == ICM42688P_WHOAMI;
}

int icm42688_decode_snapshot(const uint8_t *data, unsigned size,
                           int32_t accel[3], int32_t gyro[3])
{
    unsigned axis;
    if (!data || !accel || !gyro || size < ICM42688_SNAPSHOT_BYTES) return 0;
    for (axis = 0; axis < 3; ++axis) {
        uint32_t a = ((uint32_t)data[2 + 2 * axis] << 8) | data[3 + 2 * axis];
        uint32_t g = ((uint32_t)data[8 + 2 * axis] << 8) | data[9 + 2 * axis];
        accel[axis] = a & 0x8000U ? (int32_t)a - 65536 : (int32_t)a;
        gyro[axis] = g & 0x8000U ? (int32_t)g - 65536 : (int32_t)g;
    }
    return 1;
}

unsigned icm42688_build_fifo_config(const icm42688_config *config,
                                  sensor_register_setting *steps, unsigned capacity)
{
    /* AAF entries are the datasheet table values, not computed delt squared.
     * Analog bandwidth is a candidate; complete passband needs board testing. */
    static const struct {
        uint32_t hz;
        uint8_t odr, delt, shift;
        uint16_t square;
    } rates[] = {
        {200,7,2,13,4}, {500,15,5,10,25}, {1000,6,11,8,122},
        {2000,5,21,6,440}, {4000,4,37,4,1376},
        {8000,3,63,3,3968}, {16000,2,63,3,3968}, {32000,1,63,3,3968}
    };
    static const sensor_register_setting defaults[] = {
        {MPUREG_DEVICE_CONFIG,1,0,20000},
        {MPUREG_REG_BANK_SEL,0,7,0},
        {MPUREG_PWR_MGMT_0,0,0x3F,0},
        {MPUREG_FIFO_CONFIG,0,0xC0,0},
        {MPUREG_REG_BANK_SEL,1,7,0},
        {0x0B,0xA0,3,0}, /* gyro AAF and notch enabled */
        {0x0C,0,0x3F,0}, {0x0D,0,0xFF,0}, {0x0E,0,0xFF,0},
        {MPUREG_REG_BANK_SEL,2,7,0},
        {0x03,0,0x7F,0}, {0x04,0,0xFF,0}, {0x05,0,0xFF,0},
        {MPUREG_REG_BANK_SEL,0,7,0},
        {MPUREG_GYRO_CONFIG0,0,0xEF,0},
        {MPUREG_ACCEL_CONFIG0,0,0xEF,0},
        {MPUREG_GYRO_CONFIG1,0x12,0xEF,0}, /* first-order UI, third-order DEC2 */
        {MPUREG_ACCEL_CONFIG1,0x05,0x1F,0},
        {MPUREG_ACCEL_GYRO_CONFIG0,0,0xFF,0}, /* UI BW code 0, both axes */
        {MPUREG_INTF_CONFIG0,0x30,0xF0,0}, /* bytes, BE count/data, invalid sentinel */
        {MPUREG_TMST_CONFIG,0x21,0x1F,0}, /* preserve reserved reset bit 5 */
        {MPUREG_FIFO_CONFIG1,0x27,0x7F,0}, /* >= watermark; six axes + temp */
        {MPUREG_FIFO_CONFIG2,0,0xFF,0},
        {(MPUREG_FIFO_CONFIG2 + 1),0,0x0F,0},
        {MPUREG_INT_CONFIG,3,7,0},
        {MPUREG_INT_CONFIG1,0x60,0x70,0},
        {MPUREG_INT_SOURCE0,6,0x7F,0}, /* FIFO threshold + full */
        {MPUREG_SIGNAL_PATH_RESET,2,0,1000},
        {MPUREG_PWR_MGMT_0,0x0F,0x3F,60000}, /* settle while FIFO bypassed */
        {MPUREG_FIFO_CONFIG,0x80,0xC0,0} /* stop on full */
    };
    unsigned rate, accel, gyro, i, watermark;
    const unsigned count = sizeof(defaults) / sizeof(defaults[0]);
    if (!config || !steps || capacity < count) return 0;
    for (rate = 0; rate < sizeof(rates)/sizeof(rates[0]); ++rate)
        if (config->odr_hz == rates[rate].hz) break;
    if (rate == sizeof(rates)/sizeof(rates[0])) return 0;
    switch (config->accel_g) {
    case 16: accel = 0; break;
    case 8: accel = 1; break;
    case 4: accel = 2; break;
    case 2: accel = 3; break;
    default: return 0;
    }
    switch (config->gyro_dps) {
    case 2000: gyro = 0; break;
    case 1000: gyro = 1; break;
    case 500: gyro = 2; break;
    case 250: gyro = 3; break;
    default: return 0;
    }
    for (i = 0; i < count; ++i) steps[i] = defaults[i];
    steps[6].value = rates[rate].delt;
    steps[7].value = (uint8_t)rates[rate].square;
    steps[8].value = (uint8_t)((rates[rate].shift << 4) | (rates[rate].square >> 8));
    steps[10].value = (uint8_t)(rates[rate].delt << 1);
    steps[11].value = steps[7].value;
    steps[12].value = steps[8].value;
    steps[14].value = (uint8_t)((gyro << 5) | rates[rate].odr);
    steps[15].value = (uint8_t)((accel << 5) | rates[rate].odr);
    watermark = config->odr_hz / 500;
    if (!watermark) watermark = 1;
    /* Cap at 32 packets: each transfer <= 512 bytes; re-query for backlog. */
    if (watermark > 32) watermark = 32;
    watermark *= ICM42688_FIFO_PACKET_BYTES;
    steps[22].value = (uint8_t)watermark;
    steps[23].value = (uint8_t)(watermark >> 8);
    return count;
}

int icm42688_decode_fifo(const uint8_t *data, unsigned size, icm42688_fifo_sample *sample)
{
    unsigned axis;
    if (!data || !sample || size < ICM42688_FIFO_PACKET_BYTES) return 0;
    if ((data[0] & 0xFC) != 0x68) return 0;
    for (axis = 0; axis < 6; ++axis) {
        if (data[1 + 2 * axis] == 0x80 && data[2 + 2 * axis] == 0) return 0;
    }
    for (axis = 0; axis < 3; ++axis) {
        uint32_t a = ((uint32_t)data[1 + 2 * axis] << 8) | data[2 + 2 * axis];
        uint32_t g = ((uint32_t)data[7 + 2 * axis] << 8) | data[8 + 2 * axis];
        sample->accel[axis] = (int16_t)(a & 0x8000U ? (int32_t)a - 65536 : (int32_t)a);
        sample->gyro[axis] = (int16_t)(g & 0x8000U ? (int32_t)g - 65536 : (int32_t)g);
    }
    sample->temperature = (int8_t)(data[13] & 0x80U ? (int)data[13] - 256 : data[13]);
    sample->timestamp = (uint16_t)(((uint16_t)data[14] << 8) | data[15]);
    sample->odr_changed = data[0] & 3;
    return 1;
}
