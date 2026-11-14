/**
 * @file probe_capture_check.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "probe_acquisition.h"
#include "probe_capture_check.h"
#include "probe_time.h"
#include "SEGGER_RTT.h"
#define LOG_TAG "CAPTURE"
#include "vibra_log.h"

typedef struct {
    probe_acquisition_config config;
    uint64_t report_us;
    uint32_t consumed[2], checksum[2];
    uint8_t restart, consume;
} capture_check_context;
static capture_check_context check;

static void request_start(void)
{
    probe_acquisition_stop();
    check.restart = 1;
}

static void handle_key(int key)
{
    static const unsigned icm_rates[] = {200,500,1000,2000,4000,8000,16000,32000};
    unsigned i;
    switch (key) {
    case 'x':
        check.restart = 0;
        probe_acquisition_stop();
        LOG_I("STOP requested: diagnostic abort discards hardware FIFO tail");
        return;
    case 'q':
        check.consume = !check.consume;
        LOG_I("raw consumer=%u; off deliberately tests bounded-queue loss", check.consume);
        return;
    case 's': case 'r': request_start(); return;
    case '1': check.config.adxl.odr_hz = 4000; check.config.icm.odr_hz = 8000; break;
    case '2': check.config.adxl.odr_hz = 250; check.config.icm.odr_hz = 200; break;
    case '3': check.config.icm.odr_hz = 16000; break;
    case '4': check.config.icm.odr_hz = 32000; break;
    case 'a':
        check.config.adxl.odr_hz = check.config.adxl.odr_hz == 4000 ? 250 : check.config.adxl.odr_hz * 2;
        break;
    case 'i':
        for (i = 0; i < 8; ++i) if (icm_rates[i] == check.config.icm.odr_hz) break;
        check.config.icm.odr_hz = icm_rates[(i + 1) % 8];
        break;
    case 'd': check.config.adxl.range_g = check.config.adxl.range_g == 40 ? 10 : check.config.adxl.range_g * 2; break;
    case 'e': check.config.icm.accel_g = check.config.icm.accel_g == 16 ? 2 : check.config.icm.accel_g * 2; break;
    case 'g': check.config.icm.gyro_dps = check.config.icm.gyro_dps == 2000 ? 250 : check.config.icm.gyro_dps * 2; break;
    default: return;
    }
    request_start(); /* Never change a live chip's range/ODR in-place. */
}

void probe_capture_check_init(void)
{
    memset(&check, 0, sizeof(check));
    check.config.adxl.odr_hz = 4000;
    check.config.adxl.range_g = 10;
    check.config.icm.odr_hz = 8000;
    check.config.icm.accel_g = 16;
    check.config.icm.gyro_dps = 500;
    check.consume = check.restart = 1;
    LOG_I("M3 dual FIFO capture check; no wire protocol or attitude");
    LOG_I("s/r=start/restart x=abort q=toggle consumer; 1=default 2=slow 3=16k 4=32k");
    LOG_I("a/i=cycle ADXL/ICM ODR; d/e/g=cycle ADXL/ICM accel/gyro range");
    LOG_I("timestamps are uncalibrated anchors; raw FIFO preserved; LEDs remain off");
}

void probe_capture_check_poll(void)
{
    probe_acquisition_status status[2];
    uint64_t now = probe_time_us();
    unsigned bus, i;
    handle_key(SEGGER_RTT_GetKey());
    for (bus = 0; bus < 2; ++bus) probe_acquisition_get_status((probe_spi_bus)bus, &status[bus]);
    if (check.restart && status[0].stopped && status[1].stopped) {
        check.restart = 0;
        memset(check.consumed, 0, sizeof(check.consumed));
        memset(check.checksum, 0, sizeof(check.checksum));
        LOG_I("START ADXL=%uHz +/-%ug ICM=%uHz +/-%ug +/- %udps accepted=%u",
              check.config.adxl.odr_hz, check.config.adxl.range_g,
              check.config.icm.odr_hz, check.config.icm.accel_g, check.config.icm.gyro_dps,
              probe_acquisition_start(&check.config));
    }
    if (check.consume) {
        /* Bounded: at most one batch per sensor per turn. Future consumers
         * replace this checksum sink; no sample-by-sample RTT printing. */
        for (bus = 0; bus < 2; ++bus) {
            const probe_raw_batch *batch = probe_acquisition_peek((probe_spi_bus)bus);
            if (!batch) continue;
            for (i = 0; i < batch->size; ++i) check.checksum[bus] = check.checksum[bus] * 33U + batch->data[i];
            check.consumed[bus] += batch->samples;
            probe_acquisition_release((probe_spi_bus)bus);
        }
    }
    if (now < check.report_us) return;
    check.report_us = now + 1000000;
    for (bus = 0; bus < 2; ++bus) {
        probe_acquisition_get_status((probe_spi_bus)bus, &status[bus]);
        LOG_I("%s run=%u stop=%u samples=%u batches=%u seg=%u gap=%u fault=%u reg=%02x expected=%02x actual=%02x",
              bus ? "ICM" : "ADXL", status[bus].running, status[bus].stopped,
              status[bus].samples, status[bus].batches, status[bus].segment,
              status[bus].unknown_gaps, status[bus].last_fault,
              status[bus].fault_reg, status[bus].expected, status[bus].actual);
        LOG_I("%s spi=%u ovr=%u fmt=%u qdrop=%u fifo_max_B=%u queue_max=%u irq=%u warmup=%u poll_gap_max_us=%u",
              bus ? "ICM" : "ADXL", status[bus].spi_errors, status[bus].overflows,
              status[bus].format_errors, status[bus].queue_drops, status[bus].fifo_highwater,
              status[bus].queue_highwater, status[bus].irq_count, status[bus].discarded_startup, status[bus].max_poll_gap_us);
        LOG_I("%s a=%d,%d,%d g=%d,%d,%d tmst16=%u consumed=%u hash=%08x",
              bus ? "ICM" : "ADXL", status[bus].accel[0], status[bus].accel[1], status[bus].accel[2],
              status[bus].gyro[0], status[bus].gyro[1], status[bus].gyro[2],
              status[bus].icm_timestamp, check.consumed[bus], check.checksum[bus]);
    }
}
