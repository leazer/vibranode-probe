/**
 * @file test_probe_acquisition.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "at32f413.h"
#include "probe_acquisition.h"
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

typedef struct {
    uint8_t regs[2][3][128], bank[2], rx[2][1025];
    uint64_t now, gyro_on_us;
    unsigned early_gyro_off;
    unsigned fifo_reads[2], temperature_reads;
    uint8_t torn_temperature, fail_temperature;
    uint16_t timestamp;
    uint8_t fail[2], overflow[2], malformed[2], overflow_on_data[2];
    uint8_t realtime, no_icm_irq, real_started;
    uint64_t real_start;
    uint32_t real_delivered, real_period_us, real_tick_us;
} fake_hardware;
static fake_hardware fake;
/* Provided by tests/stubs/at32f413.h externs (TMR emulation globals). */
uint32_t fake_counter, fake_pending, fake_primask, fake_wrap_on_read;
uint64_t probe_time_us(void) { return fake.now; }
void exint_flag_clear(uint32_t flags) { (void)flags; }
probe_spi_result probe_spi_start(probe_spi_bus bus, const uint8_t *tx, uint16_t size, uint32_t timeout)
{
    unsigned reg = bus == PROBE_SPI_ADXL ? tx[0] >> 1 : tx[0] & 127;
    unsigned read = bus == PROBE_SPI_ADXL ? tx[0] & 1 : tx[0] & 128;
    unsigned i;
    (void)timeout;
    if (!read) {
        if (bus == PROBE_SPI_ICM && fake.bank[bus] == 0 && reg == 0x4E) {
            if (tx[1] == 0x0F) {
                fake.gyro_on_us = fake.now;
                fake.real_start = fake.now; fake.real_delivered = 0; fake.real_started = 0;
            }
            else if (fake.regs[bus][0][reg] == 0x0F && fake.now - fake.gyro_on_us < 45000)
                ++fake.early_gyro_off;
        }
        if (bus == PROBE_SPI_ICM && reg == 0x76) fake.bank[bus] = tx[1];
        else fake.regs[bus][fake.bank[bus]][reg] = tx[1];
    } else {
        memset(fake.rx[bus], 0, sizeof(fake.rx[bus]));
        if (bus == PROBE_SPI_ADXL && reg == 6) ++fake.temperature_reads;
        if (bus == PROBE_SPI_ICM && reg == 0x76) fake.rx[bus][1] = fake.bank[bus];
        else for (i = 1; i < size; ++i) {
            if (reg + i - 1 < 128) fake.rx[bus][i] = fake.regs[bus][fake.bank[bus]][reg+i-1];
        }
        if (bus == PROBE_SPI_ADXL && reg == 4) {
            fake.rx[bus][1] = fake.overflow[bus] ? 4 : 0;
            fake.rx[bus][2] = 24;
        }
        if (bus == PROBE_SPI_ICM && reg == 0x2D) {
            fake.rx[bus][1] = fake.overflow[bus] ? 2 : 0;
            fake.rx[bus][2] = 1; /* 256 bytes */
            fake.rx[bus][3] = 0;
            if (fake.realtime) {
                if (!fake.real_started) { fake.real_start = fake.now; fake.real_started = 1; }
                unsigned period = fake.real_period_us ? fake.real_period_us : 124;
                unsigned bytes = (unsigned)((fake.now - fake.real_start) / period - fake.real_delivered) * 16;
                fake.rx[bus][2] = (uint8_t)(bytes >> 8);
                fake.rx[bus][3] = (uint8_t)bytes;
            }
        }
        if ((bus == PROBE_SPI_ADXL && reg == 0x11) || (bus == PROBE_SPI_ICM && reg == 0x30)) {
            ++fake.fifo_reads[bus];
            if (fake.overflow_on_data[bus]) fake.overflow[bus] = 1;
            if (bus == PROBE_SPI_ADXL) {
                for (i = 1; i < size; i += 9) fake.rx[bus][i+2] = 1;
            } else {
                if (fake.realtime) fake.real_delivered += (size - 1) / 16;
                for (i = 1; i < size; i += 16) {
                    fake.rx[bus][i] = 0x68;
                    fake.timestamp = (uint16_t)(fake.timestamp + (fake.realtime && fake.real_tick_us ? fake.real_tick_us : 125));
                    fake.rx[bus][i+14] = (uint8_t)(fake.timestamp >> 8);
                    fake.rx[bus][i+15] = (uint8_t)fake.timestamp;
                }
            }
            if (fake.malformed[bus]) fake.rx[bus][1] = 0x80;
        }
    }
    if (read && bus == PROBE_SPI_ADXL && reg == 6 && size == 2 && fake.torn_temperature)
        fake.rx[bus][1] ^= 1; /* TEMP2 changed between reads. */
    if (read && bus == PROBE_SPI_ADXL && reg == 6 && fake.fail_temperature)
        return PROBE_SPI_BUSY;
    return PROBE_SPI_PENDING;
}
probe_spi_result probe_spi_read(probe_spi_bus bus, uint8_t command, uint16_t size, uint32_t timeout)
{
    uint8_t tx[1025] = {0}; /* Host-emulated transport fixture only. */
    tx[0] = command;
    return probe_spi_start(bus, tx, size + 1, timeout);
}
probe_spi_result probe_spi_poll(probe_spi_bus bus) { return fake.fail[bus] ? PROBE_SPI_TIMEOUT : PROBE_SPI_DONE; }
const uint8_t *probe_spi_rx(probe_spi_bus bus) { return fake.rx[bus]; }

static struct { uint32_t adxl_acc, icm_acc; } irq_clock;
static void tick(unsigned n, int drain)
{
    unsigned i, bus;
    for (i = 0; i < n; ++i) {
        fake.now += 100;
        /* Consistent EXINT anchor intervals (multiples of the 100 us step)
         * keep the estimator's consistency check meaningful, as a real DRDY
         * would; exact sensor rates are a board-level concern. */
        irq_clock.adxl_acc += 100;
        if (irq_clock.adxl_acc >= 300) {
            irq_clock.adxl_acc -= 300;
            probe_acquisition_irq(0);
        }
        irq_clock.icm_acc += 100;
        if (irq_clock.icm_acc >= 1400) {
            irq_clock.icm_acc -= 1400;
            if (!fake.no_icm_irq) probe_acquisition_irq(2);
        }
        probe_acquisition_poll();
        if (drain) for (bus = 0; bus < 2; ++bus)
            while (probe_acquisition_peek((probe_spi_bus)bus)) probe_acquisition_release((probe_spi_bus)bus);
    }
}
int test_main(void)
{
    probe_acquisition_config config = {{4000,10,0},{8000,16,500}};
    probe_acquisition_status a, b;
    const probe_raw_batch *batch;
    unsigned i;
    fake.regs[0][0][0] = 0xAD; fake.regs[0][0][1] = 0x1D; fake.regs[0][0][2] = 0xED;
    fake.regs[1][0][0x75] = 0x47;
    fake.regs[0][0][6] = 0x07; fake.regs[0][0][7] = 0x5D; /* 1885 = nominal 25 C */
    probe_acquisition_init();
    CHECK(probe_acquisition_start(&config));
    CHECK(!probe_acquisition_start(&config));
    tick(15000, 1); /* Includes eight 100 ms ICM FIFO-count observations. */
    probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
    probe_acquisition_get_status(PROBE_SPI_ICM, &b);
    CHECK(a.samples > 0 && b.samples > 0 && !a.last_fault && !b.last_fault);
    {
        probe_temperature temp;
        probe_acquisition_get_temperature(&temp);
        CHECK(temp.valid && temp.live && temp.raw == 1885 && temp.time_us > 0);
        /* Less than one second cannot refresh the low-rate cached reading. */
        fake.regs[0][0][7] = 0x20;
        tick(100, 1);
        probe_acquisition_get_temperature(&temp);
        CHECK(temp.raw == 1885);
        fake.regs[0][0][7] = 0x5D;
    }
    CHECK(b.tick_valid && b.tick_period_q16 > 0);
    CHECK(a.last_sequence == a.samples - 1 && a.last_read_us > 0);
    CHECK(b.last_sequence == b.samples - 1);
    tick(45, 0); /* one paced ICM read (4 ms interval): 16 samples, unmerged */
    batch = probe_acquisition_peek(PROBE_SPI_ICM);
    CHECK(batch && batch->samples == 16 && batch->size == 256);
    /* Anchors converged long ago in this run; new batches carry estimates. */
    CHECK(!(batch->flags & PROBE_BATCH_TIME_UNCERTAIN));
    CHECK(batch->first_sample_us > 0 && batch->period_q16 > 0);
    probe_acquisition_get_status(PROBE_SPI_ICM, &b);
    CHECK(b.time_valid && b.tick_valid && b.tick_period_q16 > 0);
    CHECK(!(batch->flags & PROBE_BATCH_SEGMENT_START));
    CHECK(batch->last_sensor_tick - batch->first_sensor_tick == 15 * 125);
    tick(2500, 0); /* 250 ms: bursts fill the 8 slots, then drop; the
                      segment survives the drop (round 24 policy) */
    probe_acquisition_get_status(PROBE_SPI_ICM, &b);
    CHECK(b.queue_drops > 0 && b.segment == 1);
    /* Merged slot: thirteen 8-sample ADXL reads (2 ms pacing + cycle) reach
     * the 108-sample wire cap within the 40 ms window (104 samples, 936
     * bytes); read fourteen opens a slot. */
    batch = probe_acquisition_peek(PROBE_SPI_ADXL);
    CHECK(batch && batch->samples == 104 && batch->size == 936);
    CHECK(batch->last_sensor_tick == 0);
    /* Attitude bypass keeps the per-read view even while raw reads merge. */
    batch = probe_acquisition_attitude_peek();
    CHECK(batch && batch->samples == 16 && batch->size == 256);
    probe_acquisition_attitude_release();
    tick(200, 1);
    fake.overflow_on_data[0] = 1;
    tick(20, 1);
    probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
    CHECK(a.overflows > 0);
    fake.overflow[0] = fake.overflow_on_data[0] = 0;
    fake.fail[1] = 1;
    tick(30, 1);
    probe_acquisition_get_status(PROBE_SPI_ICM, &b);
    CHECK(b.spi_errors > 0);
    fake.fail[1] = 0;
    tick(6000, 1); /* 600 ms: both estimators must have converged */
    probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
    CHECK(a.running);
    probe_acquisition_stop();
    tick(1000, 1);
    probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
    probe_acquisition_get_status(PROBE_SPI_ICM, &b);
    CHECK(a.stopped && b.stopped);
    /* Boundary follows the current segment and freezes once stopped. */
    CHECK(a.last_sequence < a.samples && a.last_read_us > 0);
    {
        uint64_t frozen = a.last_sequence;
        tick(2000, 1);
        probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
        CHECK(a.last_sequence == frozen);
    }
    CHECK(probe_acquisition_start(&config));
    tick(3500, 1);
    probe_acquisition_get_status(PROBE_SPI_ICM, &b);
    CHECK(b.samples > 0);
    fake.malformed[1] = 1;
    tick(30, 1);
    probe_acquisition_get_status(PROBE_SPI_ICM, &b);
    CHECK(b.format_errors > 0);
    fake.malformed[1] = 0;
    probe_acquisition_stop();
    tick(2000, 1);
    CHECK(probe_acquisition_start(&config));
    for (i = 0; i < 3000 && fake.regs[1][0][0x4E] != 0x0F; ++i) tick(1, 1);
    CHECK(i < 3000);
    probe_acquisition_stop(); /* Stop during gyro startup must respect on-time. */
    tick(1000, 1);
    probe_acquisition_get_status(PROBE_SPI_ICM, &b);
    CHECK(b.stopped && !fake.early_gyro_off);
    /* Real free-running 124 us sample clock, independent of poll cadence.
     * FIFO never reaches watermark; a stale IRQ must not freeze time mapping. */
    fake.realtime = fake.no_icm_irq = 1;
    CHECK(probe_acquisition_start(&config));
    tick(15000, 1);
    probe_acquisition_get_status(PROBE_SPI_ICM, &b);
    CHECK(b.running && !b.format_errors && !b.overflows);
    CHECK(b.time_valid && b.tick_valid);
    CHECK(b.tick_period_q16 > 64300 && b.tick_period_q16 < 65700);
    {
        uint64_t previous_us = 0;
        unsigned checked = 0;
        for (i = 0; i < 2000; ++i) { /* 4 ms read pacing: one batch per ~40 ticks */
            tick(1, 0);
            batch = probe_acquisition_peek(PROBE_SPI_ICM);
            if (batch) {
                CHECK(batch->period_q16 > 8040000 && batch->period_q16 < 8210000);
                CHECK(batch->first_sample_us > previous_us);
                previous_us = batch->first_sample_us;
                ++checked;
                probe_acquisition_release(PROBE_SPI_ICM);
            }
            while (probe_acquisition_peek(PROBE_SPI_ADXL)) probe_acquisition_release(PROBE_SPI_ADXL);
        }
        CHECK(checked > 20);
    }
    {
        static const uint32_t odr[] = {200, 16000, 32000};
        static const uint32_t real_period[] = {4980, 62, 31};
        unsigned rate;
        for (rate = 0; rate < 3; ++rate) {
            probe_acquisition_stop(); tick(1000, 1);
            fake.real_period_us = real_period[rate];
            fake.real_tick_us = 1000000U / odr[rate];
            config.icm.odr_hz = odr[rate];
            CHECK(probe_acquisition_start(&config));
            tick(25000, 1);
            probe_acquisition_get_status(PROBE_SPI_ICM, &b);
            CHECK(b.running && !b.format_errors && !b.overflows && b.time_valid && b.tick_valid);
        }
    }
    /* Standby pose must not run ADXL or mutate the stopped raw stream. */
    probe_acquisition_stop(); tick(1000, 1);
    probe_acquisition_get_status(PROBE_SPI_ICM, &b);
    {
        unsigned adxl_reads = fake.fifo_reads[0], taps = 0;
        probe_acquisition_status frozen = b;
        icm42688_config pose = {200, 16, 500};
        fake.real_period_us = fake.real_tick_us = 5000;
        CHECK(probe_acquisition_pose_start(&pose));
        CHECK(probe_acquisition_pose_active());
        CHECK(!probe_acquisition_start(&config));
        for (i = 0; i < 20000; ++i) {
            tick(1, 0);
            CHECK(!probe_acquisition_peek(PROBE_SPI_ADXL));
            CHECK(!probe_acquisition_peek(PROBE_SPI_ICM));
            if (probe_acquisition_attitude_peek()) {
                ++taps; probe_acquisition_attitude_release();
            }
        }
        CHECK(taps > 100 && fake.fifo_reads[0] == adxl_reads);
        probe_acquisition_get_status(PROBE_SPI_ICM, &b);
        CHECK(b.samples == frozen.samples && b.queue_drops == frozen.queue_drops);
        CHECK(b.last_sequence == frozen.last_sequence && b.last_read_us == frozen.last_read_us);
        CHECK(b.stopped && !b.running);
        probe_acquisition_stop(); tick(1000, 0);
        CHECK(!probe_acquisition_pose_active());
        CHECK(!probe_acquisition_attitude_peek());
        probe_acquisition_get_status(PROBE_SPI_ICM, &b);
        CHECK(b.samples == frozen.samples && b.last_sequence == frozen.last_sequence);
        config.icm.odr_hz = 8000;
        fake.real_period_us = fake.real_tick_us = 125;
        CHECK(probe_acquisition_start(&config));
        tick(4000, 1);
        probe_acquisition_get_status(PROBE_SPI_ICM, &b);
        CHECK(b.running && b.samples > 0 && b.segment == 1 && !b.queue_drops);
    }
    /* Pose may not steal undelivered STOP tail batches. */
    tick(500, 0);
    probe_acquisition_stop(); tick(1000, 0);
    CHECK(probe_acquisition_peek(PROBE_SPI_ICM));
    {
        icm42688_config pose = {200, 16, 500};
        CHECK(!probe_acquisition_pose_start(&pose));
        while (probe_acquisition_peek(PROBE_SPI_ADXL)) probe_acquisition_release(PROBE_SPI_ADXL);
        while (probe_acquisition_peek(PROBE_SPI_ICM)) probe_acquisition_release(PROBE_SPI_ICM);
        CHECK(probe_acquisition_pose_start(&pose));
    }
    /* Independent, low-rate reads must not disturb FIFO sequence/status. */
    probe_acquisition_stop(); tick(1000, 1);
    fake.torn_temperature = 0;
    CHECK(probe_acquisition_start(&config));
    tick(15000, 1);
    {
        probe_temperature temp;
        unsigned reads = fake.temperature_reads;
        probe_acquisition_get_temperature(&temp);
        CHECK(temp.valid && temp.live && temp.raw == 1885);
        fake.torn_temperature = 1;
        tick(11000, 1);
        probe_acquisition_get_temperature(&temp);
        CHECK(!temp.valid && fake.temperature_reads <= reads + 4);
        probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
        CHECK(a.running && !a.spi_errors && !a.overflows && a.segment == 1);
        fake.torn_temperature = 0; fake.fail_temperature = 1;
        tick(11000, 1);
        probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
        CHECK(a.running && !a.spi_errors && a.segment == 1);
        fake.fail_temperature = 0; tick(11000, 1);
        probe_acquisition_get_temperature(&temp);
        CHECK(temp.valid && temp.live);
        probe_acquisition_stop(); tick(1000, 1);
        probe_acquisition_get_temperature(&temp);
        CHECK(temp.valid && !temp.live && temp.raw == 1885);
    }
    return 0;
}
