/**
 * @file test_sensor_fifo.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "adxl357.h"
#include "icm42688.h"
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

int test_main(void)
{
    sensor_register_setting steps[40];
    adxl357_config adxl = {4000, 10, 0};
    icm42688_config icm = {8000, 16, 500};
    uint8_t a[9] = {0x80,0,1, 0x7F,0xFF,0xF0, 0,0,0x10};
    uint8_t p[16] = {0x68,0xFF,0xFF,0,1,0,2,0,3,0,4,0,5,0xFE,0xFF,0xF0};
    int32_t xyz[3];
    icm42688_fifo_sample sample;
    unsigned i, count;
    const unsigned a_rates[] = {250,500,1000,2000,4000};
    const unsigned i_rates[] = {200,500,1000,2000,4000,8000,16000,32000};
    CHECK(adxl357_build_fifo_config(&adxl, steps, 40) > 0);
    CHECK(icm42688_build_fifo_config(&icm, steps, 40) > 0);
    CHECK(steps[5].value == 0xA0); /* GYRO_CONFIG_STATIC2 reserved reset bits */
    CHECK(steps[16].value == 0x12); /* GYRO_CONFIG1 reserved bit 4 */
    CHECK(!adxl357_build_fifo_config(&adxl, steps, 1));
    CHECK(!icm42688_build_fifo_config(&icm, steps, 1));
    for (i = 0; i < 5; ++i) {
        adxl.odr_hz = a_rates[i];
        CHECK(adxl357_build_fifo_config(&adxl, steps, 40) > 0);
    }
    for (i = 0; i < 8; ++i) {
        icm.odr_hz = i_rates[i];
        count = icm42688_build_fifo_config(&icm, steps, 40);
        CHECK(count > 0 && count <= 40);
    }
    adxl.odr_hz = 4000;
    for (i = 10; i <= 40; i *= 2) {
        adxl.range_g = (uint16_t)i;
        CHECK(adxl357_build_fifo_config(&adxl, steps, 40) > 0);
    }
    for (i = 0; i <= 6; ++i) {
        adxl.hpf = (uint8_t)i;
        CHECK(adxl357_build_fifo_config(&adxl, steps, 40) > 0);
    }
    adxl.hpf = 7;
    CHECK(!adxl357_build_fifo_config(&adxl, steps, 40));
    adxl.hpf = 0;
    icm.odr_hz = 8000;
    for (i = 2; i <= 16; i *= 2) {
        icm.accel_g = (uint16_t)i;
        CHECK(icm42688_build_fifo_config(&icm, steps, 40) > 0);
    }
    for (i = 250; i <= 2000; i *= 2) {
        icm.gyro_dps = (uint16_t)i;
        CHECK(icm42688_build_fifo_config(&icm, steps, 40) > 0);
    }
    adxl.odr_hz = 125;
    CHECK(!adxl357_build_fifo_config(&adxl, steps, 40));
    icm.odr_hz = 100;
    CHECK(!icm42688_build_fifo_config(&icm, steps, 40));
    icm.odr_hz = 8000; icm.gyro_dps = 125;
    CHECK(!icm42688_build_fifo_config(&icm, steps, 40));
    CHECK(adxl357_decode_fifo_xyz(a, 9, xyz));
    CHECK(xyz[0] == -524288 && xyz[1] == 524287 && xyz[2] == 1);
    CHECK(!adxl357_decode_fifo_xyz(a, 8, xyz));
    a[5] |= 1;
    CHECK(!adxl357_decode_fifo_xyz(a, 9, xyz));
    a[5] = 0xF2;
    CHECK(!adxl357_decode_fifo_xyz(a, 9, xyz));
    CHECK(icm42688_decode_fifo(p, 16, &sample));
    CHECK(sample.accel[0] == -1 && sample.gyro[2] == 5);
    CHECK(sample.temperature == -2 && sample.timestamp == 65520);
    CHECK(!icm42688_decode_fifo(p, 15, &sample));
    p[0] = 0x80;
    CHECK(!icm42688_decode_fifo(p, 16, &sample));
    p[0] = 0x78;
    CHECK(!icm42688_decode_fifo(p, 16, &sample));
    p[0] = 0x68; p[1] = 0x80; p[2] = 0;
    CHECK(!icm42688_decode_fifo(p, 16, &sample));
    return 0;
}
