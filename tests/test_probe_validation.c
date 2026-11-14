/**
 * @file test_probe_validation.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <stdarg.h>
#include <string.h>
#include "at32f413.h"
#include "probe_spi.h"
#include "probe_validation.h"
#include "adxl357.h"
#include "icm42688.h"

typedef struct {
    uint8_t regs[2][128], rx[2][16];
    uint64_t now;
    unsigned reads[2], config_pass, failures, reports;
    unsigned writes[2], write_mismatch;
    int32_t accel[2][3], gyro[3];
    int key, fail_icm;
} fake_validation_hardware;
static fake_validation_hardware fake;

void test_log_info(const char *format, ...)
{
    va_list args;
    if (strstr(format, "CONFIG PASS")) ++fake.config_pass;
    if (!strstr(format, "a_raw=")) return;
    va_start(args, format);
    {
        const char *name = va_arg(args, const char *);
        unsigned bus = name[0] == 'A' ? 0 : 1, axis;
        (void)va_arg(args, unsigned); /* state */
        (void)va_arg(args, unsigned); /* samples */
        (void)va_arg(args, unsigned); /* errors */
        for (axis = 0; axis < 3; ++axis) fake.accel[bus][axis] = va_arg(args, int);
        for (axis = 0; axis < 3; ++axis) {
            int value = va_arg(args, int);
            if (bus == 1) fake.gyro[axis] = value;
        }
        ++fake.reports;
    }
    va_end(args);
}
void test_log_error(const char *format, ...) { (void)format; ++fake.failures; }
uint64_t probe_time_us(void) { return fake.now; }
int SEGGER_RTT_GetKey(void) { int key = fake.key; fake.key = -1; return key; }
void gpio_bits_set(gpio_type *port, uint16_t bits) { (void)port; (void)bits; }
void gpio_bits_reset(gpio_type *port, uint16_t bits) { (void)port; (void)bits; }
void exint_flag_clear(uint32_t flags) { (void)flags; }
void nvic_irq_enable(IRQn_Type irq, uint32_t preempt, uint32_t sub) { (void)irq; (void)preempt; (void)sub; }
void probe_spi_init(void) {}
const uint8_t *probe_spi_rx(probe_spi_bus bus) { return fake.rx[bus]; }
probe_spi_result probe_spi_poll(probe_spi_bus bus)
{
    return bus == PROBE_SPI_ICM && fake.fail_icm ? PROBE_SPI_TIMEOUT : PROBE_SPI_DONE;
}
probe_spi_result probe_spi_start(probe_spi_bus bus, const uint8_t *tx, uint16_t size, uint32_t timeout)
{
    unsigned reg = bus == PROBE_SPI_ADXL ? tx[0] >> 1 : tx[0] & 0x7F;
    unsigned read = bus == PROBE_SPI_ADXL ? tx[0] & 1 : tx[0] & 0x80;
    unsigned i;
    (void)timeout;
    if (!read) {
        /* Golden sequence captured from M2, independent of component tables. */
        static const uint8_t adxl_writes[][2] = {
            {0x2F,0x52}, {0x2D,1}, {0x28,0}, {0x2C,0xC1}, {0x2A,1}, {0x2D,0}
        };
        static const uint8_t icm_writes[][2] = {
            {0x11,1}, {0x76,0}, {0x4F,0x43}, {0x50,3},
            {0x14,3}, {0x64,0x60}, {0x65,8}, {0x4E,0x0F}
        };
        const uint8_t *expected = bus == PROBE_SPI_ADXL ?
            adxl_writes[fake.writes[bus] % 6] : icm_writes[fake.writes[bus] % 8];
        if (size != 2 || reg != expected[0] || tx[1] != expected[1]) ++fake.write_mismatch;
        ++fake.writes[bus];
        fake.regs[bus][reg] = tx[1];
    }
    else {
        for (i = 1; i < size; ++i) fake.rx[bus][i] = fake.regs[bus][reg+i-1];
        if ((bus == PROBE_SPI_ADXL && reg == 4) || (bus == PROBE_SPI_ICM && reg == 0x1D)) ++fake.reads[bus];
    }
    return PROBE_SPI_PENDING;
}

#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
int test_main(void)
{
    unsigned i, previous;
    fake.regs[0][0] = 0xAD; fake.regs[0][1] = 0x1D; fake.regs[0][2] = 0xED;
    fake.regs[1][0x75] = 0x47;
    fake.regs[0][8] = 0x80;
    fake.regs[0][11] = 0x7F; fake.regs[0][12] = 0xFF; fake.regs[0][13] = 0xF0;
    fake.regs[0][16] = 0x10;
    fake.regs[1][0x1F] = 0xFB; fake.regs[1][0x20] = 0x2E; /* -1234 */
    fake.regs[1][0x21] = 0x09; fake.regs[1][0x22] = 0x29; /* +2345 */
    fake.regs[1][0x28] = 1; /* gyro Y = 1 */
    fake.key = -1;
    probe_validation_init();
    for (i = 0; i < 2200; ++i) {
        fake.now += 1000;
        probe_validation_poll();
    }
    CHECK(fake.config_pass == 2 && fake.failures == 0);
    CHECK(fake.writes[0] == 6 && fake.writes[1] == 8 && fake.write_mismatch == 0);
    CHECK(!adxl357_decode_snapshot(fake.rx[0], 12, fake.accel[0]));
    CHECK(!icm42688_decode_snapshot(fake.rx[1], 13, fake.accel[1], fake.gyro));
    CHECK(!adxl357_identity_valid(0, 4) && !icm42688_identity_valid(0, 1));
    CHECK(fake.reads[0] > 100 && fake.reads[1] > 100);
    CHECK(fake.reports >= 4);
    CHECK(fake.accel[0][0] == -524288 && fake.accel[0][1] == 524287 && fake.accel[0][2] == 1);
    CHECK(fake.accel[1][0] == -1234 && fake.accel[1][1] == 2345);
    CHECK(fake.gyro[1] == 1);
    fake.fail_icm = 1;
    previous = fake.reads[0];
    for (i = 0; i < 100; ++i) { fake.now += 1000; probe_validation_poll(); }
    CHECK(fake.failures == 1 && fake.reads[0] > previous);
    fake.fail_icm = 0;
    fake.key = 'r';
    for (i = 0; i < 400; ++i) { fake.now += 1000; probe_validation_poll(); }
    CHECK(fake.config_pass == 4);
    CHECK(fake.writes[0] == 12 && fake.writes[1] == 16 && fake.write_mismatch == 0);
    /* Identity mismatch must stop before applying settings. */
    fake.regs[1][0x75] = 0;
    fake.key = 'r';
    previous = fake.config_pass;
    for (i = 0; i < 400; ++i) { fake.now += 1000; probe_validation_poll(); }
    CHECK(fake.config_pass == previous + 1 && fake.failures == 2);
    return 0;
}
