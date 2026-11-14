/**
 * @file test_probe_attitude.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <math.h>
#include <string.h>
#include "at32f413.h"
#include "probe_spi.h"
#include "probe_attitude.h"
#include "probe_acquisition.h"

#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

static uint64_t fake_now;
static probe_raw_batch batch;
static int queued;
const probe_raw_batch *probe_acquisition_attitude_peek(void) { return queued ? &batch : 0; }
void probe_acquisition_attitude_release(void) { queued = 0; }
uint64_t probe_time_us(void) { return fake_now; }

/* Acquisition transport fakes: probe_attitude only reads the bypass queue,
 * which stays empty in these tests. */
void exint_flag_clear(uint32_t flags) { (void)flags; }
void nvic_irq_enable(IRQn_Type irq, uint32_t p, uint32_t s) { (void)irq; (void)p; (void)s; }
probe_spi_result probe_spi_start(probe_spi_bus bus, const uint8_t *tx, uint16_t size, uint32_t timeout)
{ (void)bus; (void)tx; (void)size; (void)timeout; return PROBE_SPI_DONE; }
probe_spi_result probe_spi_read(probe_spi_bus bus, uint8_t command, uint16_t size, uint32_t timeout)
{ (void)bus; (void)command; (void)size; (void)timeout; return PROBE_SPI_DONE; }
probe_spi_result probe_spi_poll(probe_spi_bus bus) { (void)bus; return PROBE_SPI_DONE; }
const uint8_t *probe_spi_rx(probe_spi_bus bus) { (void)bus; return 0; }

static const icm42688_fifo_sample *make_sample(float amp_counts, float freq, double t_us, int static_g)
{
    static icm42688_fifo_sample s;
    double value = amp_counts * sin(2.0 * 3.14159265358979 * freq * t_us / 1e6);
    int axis;
    s.temperature = 25;
    s.timestamp = 0;
    for (axis = 0; axis < 3; ++axis) {
        s.accel[axis] = (int16_t)value;
        s.gyro[axis] = (int16_t)value;
    }
    s.accel[2] = (int16_t)(2048 * static_g + value);
    return &s;
}

/* Run one tone through the chain; reports the peak/valley of the 200 Hz
 * output X channel once the filters have warmed up. */
static void run_tone(const icm42688_config *config, unsigned seconds, float amp, float freq, int static_g,
                     int16_t *peak, int16_t *valley)
{
    unsigned i, n = seconds * config->odr_hz;
    double t = 0;
    icm42688_fifo_sample batch[16];
    probe_attitude_status st;
    *peak = 0;
    *valley = 0;
    for (i = 0; i < n; ++i) {
        unsigned slot = i % 16;
        batch[slot] = *make_sample(amp, freq, t, static_g);
        t += 1000000.0 / config->odr_hz;
        if (slot == 15 || i + 1 == n) {
            probe_attitude_ingest(batch, slot + 1,
                                  (uint64_t)t - (uint64_t)(1000000U / config->odr_hz) * (slot + 1),
                                  (uint32_t)(((uint64_t)1000000 << 16) / config->odr_hz), 0x0001U);
            probe_attitude_get_status(&st);
            if (st.last_output[0] > *peak) *peak = st.last_output[0];
            if (st.last_output[0] < *valley) *valley = st.last_output[0];
        }
    }
}

int test_main(void)
{
    icm42688_config config = {8000, 16, 500};
    probe_attitude_status st;
    int16_t peak, valley;

    probe_attitude_init();
    CHECK(probe_attitude_start(&config) == 1);

    /* Passband: a 10 Hz tone survives within +-2 dB (1000 counts in). */
    run_tone(&config, 2, 1000, 10.0f, 1, &peak, &valley);
    if (peak <= 794) return 700 + peak / 10; /* TEMP */
    if (peak >= 1259) return 800 + peak / 10; /* TEMP */
    CHECK(valley < -794 && valley > -1259);

    /* Restart for a clean filter state; a 150 Hz tone aliases to 50 Hz at the
     * output and must be at least ~54 dB down. */
    CHECK(probe_attitude_start(&config) == 1);
    run_tone(&config, 2, 1000, 150.0f, 1, &peak, &valley);
    if (peak < 0) peak = (int16_t)(-peak);
    if (valley > 0) valley = (int16_t)(-valley);
    CHECK(peak < 3 && valley > -3);

    /* Static +1 g / zero rate: AGM must converge to a valid unit quaternion. */
    CHECK(probe_attitude_start(&config) == 1);
    run_tone(&config, 3, 0, 10.0f, 1, &peak, &valley);
    probe_attitude_get_status(&st);
    CHECK(st.quat_valid && st.processed > 400);
    CHECK(st.queued_outputs == 8);
    CHECK(st.dropped_outputs == st.processed - st.queued_outputs);
    {
        const probe_attitude_sample *sample = probe_attitude_peek();
        uint64_t expected = 0;
        unsigned drained = 0;
        while (sample) {
            CHECK(sample->sequence == expected++);
            CHECK(sample->flags == 0x0001U);
            CHECK(sample->time_us > 0);
            ++drained;
            probe_attitude_release();
            sample = probe_attitude_peek();
        }
        CHECK(drained == 8);
    }
    {
        double norm2 = 0;
        int axis;
        for (axis = 0; axis < 4; ++axis) {
            double v = (double)st.quat_q30[axis] / 1073741824.0;
            norm2 += v * v;
        }
        CHECK(norm2 > 0.98 && norm2 < 1.02);
    }

    /* Every supported ICM rate must converge to AGM's uniform 200 Hz input.
     * The full spectral checks above exercise the common FIR implementation;
     * this bounded sweep also keeps the ARM instruction-level test practical. */
    {
        static const unsigned rates[] = {200, 500, 1000, 2000, 4000, 8000, 16000, 32000};
        unsigned i;
        for (i = 0; i < sizeof(rates) / sizeof(rates[0]); ++i) {
            config.odr_hz = rates[i];
            CHECK(probe_attitude_start(&config) == 1);
            run_tone(&config, 1, 0, 10.0f, 1, &peak, &valley);
            probe_attitude_get_status(&st);
            CHECK(st.quat_valid && st.processed > 50 && st.samples_in == rates[i]);
        }
        config.odr_hz = 123;
        CHECK(probe_attitude_start(&config) == 0);
        queued = 1;
        probe_attitude_poll();
        CHECK(!queued); /* inactive branches still drain the borrowed queue */
    }
    /* Idle UI gets the newest orientation even after FETCH pauses. */
    config.odr_hz = 200;
    CHECK(probe_attitude_start(&config));
    probe_attitude_latest_only(1);
    run_tone(&config, 3, 0, 10.0f, 1, &peak, &valley);
    probe_attitude_get_status(&st);
    CHECK(st.quat_valid && st.queued_outputs == 1 && st.processed > 400);
    CHECK(probe_attitude_peek()->sequence == st.processed - 1);
    CHECK(probe_attitude_peek()->time_us > 2500000);
    return 0;
}
