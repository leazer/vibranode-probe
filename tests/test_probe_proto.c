/**
 * @file test_probe_proto.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include <stdarg.h>
#include "at32f413.h"
#include "probe_acquisition.h"
#include "probe_frame.h"
#include "probe_usb.h"
#include "icm42688.h"
#include "probe_proto.h"
#include "probe_attitude.h"
#include "probe_calib.h"
#define LOG_TAG "TEST"
#include "vibra_log.h"
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

void test_log_info(const char *fmt, ...) { (void)fmt; }
void test_log_error(const char *fmt, ...) { (void)fmt; }

/* ---- probe_spi fakes (shared with the acquisition test) ---- */
static struct {
    uint8_t regs[2][3][128], bank[2], rx[2][1025];
    uint64_t now, gyro_on_us;
    uint16_t timestamp;
    uint8_t fail[2], overflow[2], malformed[2], overflow_on_data[2];
} fake;
uint64_t probe_time_us(void) { return fake.now; }
void exint_flag_clear(uint32_t flags) { (void)flags; }
void nvic_irq_enable(IRQn_Type irq, uint32_t p, uint32_t s) { (void)irq; (void)p; (void)s; }
probe_spi_result probe_spi_start(probe_spi_bus bus, const uint8_t *tx, uint16_t size, uint32_t timeout)
{
    unsigned reg = bus == PROBE_SPI_ADXL ? tx[0] >> 1 : tx[0] & 127;
    unsigned read = bus == PROBE_SPI_ADXL ? tx[0] & 1 : tx[0] & 128;
    unsigned i;
    (void)timeout;
    if (!read) {
        if (bus == PROBE_SPI_ICM && fake.bank[bus] == 0 && reg == 0x4E && tx[1] == 0x0F)
            fake.gyro_on_us = fake.now;
        if (bus == PROBE_SPI_ICM && reg == 0x76) fake.bank[bus] = tx[1];
        else fake.regs[bus][fake.bank[bus]][reg] = tx[1];
    } else {
        memset(fake.rx[bus], 0, sizeof(fake.rx[bus]));
        if (bus == PROBE_SPI_ICM && reg == 0x76) fake.rx[bus][1] = fake.bank[bus];
        else for (i = 1; i < size; ++i)
            if (reg + i - 1 < 128) fake.rx[bus][i] = fake.regs[bus][fake.bank[bus]][reg + i - 1];
        if (bus == PROBE_SPI_ADXL && reg == 4) {
            fake.rx[bus][1] = fake.overflow[bus] ? 4 : 0;
            fake.rx[bus][2] = 24;
        }
        if (bus == PROBE_SPI_ICM && reg == 0x2D) {
            fake.rx[bus][1] = fake.overflow[bus] ? 2 : 0;
            fake.rx[bus][2] = 1; /* 256 bytes */
        }
        if ((bus == PROBE_SPI_ADXL && reg == 0x11) || (bus == PROBE_SPI_ICM && reg == 0x30)) {
            if (fake.overflow_on_data[bus]) fake.overflow[bus] = 1;
            if (bus == PROBE_SPI_ADXL) {
                static const uint8_t xyz[] = {
                    0xFF, 0x5D, 0x81, /* -2600 plus FIFO X marker */
                    0x00, 0xA8, 0xC0, /* 2700 */
                    0x0C, 0x22, 0x40  /* 49700 */
                };
                for (i = 1; i < size; i += 9) memcpy(fake.rx[bus] + i, xyz, sizeof(xyz));
            } else {
                for (i = 1; i < size; i += 16) {
                    fake.rx[bus][i] = 0x68;
                    fake.timestamp = (uint16_t)(fake.timestamp + ((fake.regs[1][0][0x4F] & 15) == 7 ? 5000 : 125));
                    fake.rx[bus][i + 14] = (uint8_t)(fake.timestamp >> 8);
                    fake.rx[bus][i + 15] = (uint8_t)fake.timestamp;
                }
            }
            if (fake.malformed[bus]) fake.rx[bus][1] = 0x80;
        }
    }
    return PROBE_SPI_PENDING;
}
probe_spi_result probe_spi_read(probe_spi_bus bus, uint8_t command, uint16_t size, uint32_t timeout)
{
    uint8_t tx[1025] = {0};
    tx[0] = command;
    return probe_spi_start(bus, tx, size + 1, timeout);
}
probe_spi_result probe_spi_poll(probe_spi_bus bus) { return fake.fail[bus] ? PROBE_SPI_TIMEOUT : PROBE_SPI_DONE; }
const uint8_t *probe_spi_rx(probe_spi_bus bus) { return fake.rx[bus]; }

/* ---- probe_usb fake: programmable RX queue and TX log ---- */
static struct {
    uint8_t rx[4096];
    unsigned rx_head, rx_tail;
    uint8_t tx[64][1024];
    unsigned tx_len[64];
    unsigned tx_count;
    unsigned init_count;
    unsigned blocked;
    unsigned connected;
} usb_fake;
void probe_usb_init(void) { ++usb_fake.init_count; }
unsigned probe_usb_read(uint8_t *out, unsigned capacity)
{
    unsigned moved = 0;
    while (moved < capacity && usb_fake.rx_tail != usb_fake.rx_head) {
        out[moved++] = usb_fake.rx[usb_fake.rx_tail];
        usb_fake.rx_tail = (usb_fake.rx_tail + 1) % sizeof(usb_fake.rx);
    }
    return moved;
}
int probe_usb_send(const uint8_t *frame, unsigned size, uint32_t timeout_us)
{
    (void)timeout_us;
    if (usb_fake.blocked || usb_fake.tx_count >= 64) return 0;
    memcpy(usb_fake.tx[usb_fake.tx_count], frame, size);
    usb_fake.tx_len[usb_fake.tx_count] = size;
    ++usb_fake.tx_count;
    return 1;
}
void probe_usb_poll(void) { }
int probe_usb_tx_busy(void) { return usb_fake.blocked != 0; }
uint32_t probe_usb_error_count(void) { return 0; }
unsigned probe_usb_connected(void) { return usb_fake.connected; }

/* probe_attitude fake: record start/stop. */
unsigned att_started;
static unsigned att_ready;
static probe_attitude_sample att_sample = {
    7, 1234567, {0x10000000, -0x20000000, 0x30000000, 0x40000000}, 1
};
int probe_attitude_start(const icm42688_config *config) { (void)config; att_started = 1; return 1; }
void probe_attitude_stop(void) { att_started = 0; }
void probe_attitude_init(void) { }
void probe_attitude_poll(void) { }
void probe_attitude_latest_only(int enabled) { (void)enabled; }
void probe_attitude_get_status(probe_attitude_status *st) { (void)st; }
const probe_attitude_sample *probe_attitude_peek(void) { return att_ready ? &att_sample : 0; }
void probe_attitude_release(void) { att_ready = 0; }

/* probe_calib fake: expose protocol state transitions and record encoding. */
static probe_calib_status cal_status;
static unsigned cal_save_count;
static probe_calib_record cal_record = {
    PROBE_CALIB_MAGIC, PROBE_CALIB_VERSION, PROBE_CALIB_VALID_ICM_ACCEL, 0,
    {0, 0, 0}, {1, 1, 1}, {0.01f, -0.02f, 0.03f}, {1.01f, 0.99f, 1.0f},
    {0.5f, -0.25f, 0.125f}, 26.5f, 1234, 7, 0
};
uint8_t probe_calib_begin(uint8_t target, uint8_t face,
                          const probe_acquisition_config *config, uint64_t now)
{
    (void)config; (void)now;
    cal_status.state = PROBE_CALIB_CAPTURING;
    cal_status.target = target; cal_status.face = face;
    cal_status.last_result = PROBE_CALIB_RESULT_IN_PROGRESS;
    return PROBE_CALIB_RESULT_IN_PROGRESS;
}
void probe_calib_cancel(void) { cal_status.state = PROBE_CALIB_IDLE; }
void probe_calib_reset_progress(uint8_t target) { cal_status.target = target; cal_status.completed_mask = 0; }
void probe_calib_ingest(probe_spi_bus bus, const probe_raw_batch *batch, uint64_t now)
{ (void)bus; (void)batch; (void)now; }
void probe_calib_poll(uint64_t now) { (void)now; }
void probe_calib_get_status(probe_calib_status *out) { *out = cal_status; }
void probe_calib_get_candidate(probe_calib_record *out) { *out = cal_record; }
uint8_t probe_calib_save_candidate(uint32_t created_ms)
{ ++cal_save_count; cal_record.created_ms = created_ms; return PROBE_CALIB_RESULT_OK; }

/* probe_led fake: remember the last driven work state. */
unsigned led_last_state = 999;
void probe_led_init(void) { led_last_state = 0; }
void probe_led_update(unsigned state) { led_last_state = state; }

static void reset_usb_fake(void)
{
    usb_fake.rx_head = usb_fake.rx_tail = 0;
    usb_fake.tx_count = 0;
    usb_fake.connected = 1; /* host up: the LED mirrors the wire state */
}

static void feed_request(uint8_t type, uint32_t session, uint32_t seq,
                         const uint8_t *payload, unsigned len)
{
    uint8_t frame[64];
    unsigned total = probe_frame_build(frame, type, session, seq, payload, len), i;
    for (i = 0; i < total; ++i) {
        usb_fake.rx[usb_fake.rx_head] = frame[i];
        usb_fake.rx_head = (usb_fake.rx_head + 1) % sizeof(usb_fake.rx);
    }
}

/* Decode one captured TX frame with the production parser. */
static const uint8_t *tx_payload(unsigned index, uint16_t *len)
{
    static probe_frame_parser parser;
    probe_frame_parser_init(&parser, 0);
    if (!probe_frame_parse(&parser, usb_fake.tx[index], usb_fake.tx_len[index], 0)) return 0;
    *len = parser.payload_len;
    return parser.payload;
}

static void tick(unsigned n)
{
    unsigned i;
    for (i = 0; i < n; ++i) {
        fake.now += 100;
        probe_acquisition_poll();
        probe_proto_poll();
    }
}

static void run_window(unsigned max_ticks)
{
    unsigned i, base = usb_fake.tx_count;
    for (i = 0; i < max_ticks; ++i) {
        tick(1);
        if (usb_fake.tx_count > base) {
            uint16_t len;
            const uint8_t *p = tx_payload(usb_fake.tx_count - 1, &len);
            /* FETCH_DONE is always the final frame of a window. */
            if (usb_fake.tx[usb_fake.tx_count - 1][4] == 0x14 && p && p[4] == 0) break;
        }
    }
}

static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint64_t rd64(const uint8_t *p) { return rd32(p) | ((uint64_t)rd32(p + 4) << 32); }
static void wr32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

static int find_tx(uint8_t type, unsigned from)
{
    unsigned i;
    for (i = from; i < usb_fake.tx_count; ++i)
        if (usb_fake.tx[i][4] == type) return (int)i;
    return -1;
}

int test_main(void)
{
    probe_acquisition_status st, st_i;
    uint8_t p[32];
    uint16_t len;
    int idx;
    unsigned base;

    fake.regs[0][0][0] = 0xAD; fake.regs[0][0][1] = 0x1D; fake.regs[0][0][2] = 0xED;
    fake.regs[1][0][0x75] = 0x47;
    probe_acquisition_init();
    probe_proto_init();
    CHECK(usb_fake.init_count == 1); /* Protocol startup must arm physical RX. */

    /* No session means no standby acquisition or unsolicited transmission. */
    usb_fake.connected = 1;
    tick(2000);
    CHECK(!probe_acquisition_pose_active() && !usb_fake.tx_count);

    /* a. HELLO binds the proposed session and answers with identity. */
    wr32(p, 1); wr32(p + 4, 0x11223344U);
    feed_request(0x01, 0, 0, p, 8);
    tick(1);
    CHECK(find_tx(0x01, 0) == 0);
    len = 0;
    {
        const uint8_t *r = tx_payload(0, &len);
        CHECK(r && len == 29);
        CHECK(rd32(r) == 1 && r[4] == 0); /* request id echoed, OK */
        CHECK(r[5] == 0x01 && r[6] == 0x00); /* probe type 0x0001 */
        CHECK(usb_fake.tx[0][7] == 0x44 && usb_fake.tx[0][10] == 0x11); /* session adopted */
    }

    tick(2000);
    CHECK(probe_acquisition_pose_active());
    CHECK(att_started && !probe_acquisition_peek(PROBE_SPI_ADXL) &&
          !probe_acquisition_peek(PROBE_SPI_ICM));
    probe_acquisition_get_status(PROBE_SPI_ICM, &st);
    CHECK(st.stopped && !st.samples && !st.queue_drops);

    /* b. START before any config: NOT_READY. */
    wr32(p, 2);
    feed_request(0x04, 0x11223344U, 1, p, 4);
    tick(1);
    idx = find_tx(0x04, 0);
    CHECK(idx >= 1);
    len = 0;
    {
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        CHECK(r && len == 5 && rd32(r) == 2 && r[4] == 6); /* NOT_READY */
    }

    /* c1. SET_CONFIG with an out-of-range ADXL ODR code: INVALID_PARAM. */
    wr32(p, 3); wr32(p + 4, 0x000000AB);
    p[8] = 2; p[9] = 0x01; p[10] = 5; p[11] = 0; p[12] = 0;      /* ADXL odr code 5 */
    p[13] = 0x02; p[14] = 5; p[15] = 0x13; p[16] = 0;            /* ICM 8k/16g/500 */
    feed_request(0x03, 0x11223344U, 2, p, 17);
    tick(1);
    idx = find_tx(0x03, 0);
    len = 0;
    {
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        CHECK(r && len == 5 && r[4] == 4); /* INVALID_PARAM */
    }

    /* c2. Non-default filter code: UNSUPPORTED_CONFIG naming the stream. */
    wr32(p, 4); wr32(p + 4, 0x000000AB);
    p[8] = 2; p[9] = 0x01; p[10] = 4; p[11] = 0; p[12] = 1;
    p[13] = 0x02; p[14] = 5; p[15] = 0x13; p[16] = 0;
    feed_request(0x03, 0x11223344U, 3, p, 17);
    tick(1);
    idx = find_tx(0x03, (unsigned)idx + 1); /* skip the c1 reply */
    len = 0;
    {
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        CHECK(r && len == 6 && r[4] == 5 && r[5] == 0x01); /* filter is an ADXL field */
    }

    /* c3. Valid default configuration: OK, effective values echoed. */
    wr32(p, 5); wr32(p + 4, 0x000000AB);
    p[8] = 2; p[9] = 0x01; p[10] = 4; p[11] = 0; p[12] = 0;      /* ADXL 4000 SPS +-10g HPF off */
    p[13] = 0x02; p[14] = 5; p[15] = 0x13; p[16] = 0;            /* ICM 8000 SPS accel +-16 gyro +-500 */
    feed_request(0x03, 0x11223344U, 4, p, 17);
    tick(1);
    idx = find_tx(0x03, (unsigned)idx + 1);
    len = 0;
    {
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        CHECK(r && len == 14 && r[4] == 0 && r[5] == 2);
        CHECK(r[6] == 0x01 && r[7] == 4 && r[8] == 0 && r[9] == 0);
        CHECK(r[10] == 0x02 && r[11] == 5 && r[12] == 0x13 && r[13] == 0);
    }
    /* GET_STATUS reports the config id. */
    wr32(p, 6);
    feed_request(0x09, 0x11223344U, 5, p, 4);
    tick(1);
    idx = find_tx(0x09, 0);
    len = 0;
    {
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        CHECK(r && len == 115);
        CHECK(rd32(r + 7) == 0x000000ABU);
        CHECK(r[11] == 0x01); /* stream id */
    }

    /* d. START id=7 starts; replaying the same request is served from cache;
     *    a fresh START or SET_CONFIG/SET_LINK while measuring is BUSY. */
    wr32(p, 7);
    feed_request(0x04, 0x11223344U, 6, p, 4);
    tick(1000);
    idx = find_tx(0x04, (unsigned)idx + 1); /* skip the NOT_READY reply */
    len = 0;
    {
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        CHECK(r && len == 5 && r[4] == 0);
    }
    base = usb_fake.tx_count;
    feed_request(0x04, 0x11223344U, 6, p, 4); /* exact replay */
    tick(1);
    {
        const uint8_t *r = tx_payload(base, &len);
        CHECK(usb_fake.tx_count == base + 1);
        CHECK(r && len == 5 && r[4] == 0); /* cached OK, no second side effect */
    }
    wr32(p, 8);
    feed_request(0x04, 0x11223344U, 7, p, 4);
    tick(1);
    {
        const uint8_t *r = tx_payload(base + 1, &len);
        CHECK(r && len == 5 && r[4] == 1); /* BUSY */
    }
    wr32(p, 9);
    wr32(p + 4, 1000000U);
    feed_request(0x02, 0x11223344U, 8, p, 8);
    tick(1);
    {
        /* SET_LINK is a USB-era compatibility no-op: accepted even while
         * measuring, echo only, no link state to change. */
        const uint8_t *r = tx_payload(base + 2, &len);
        CHECK(r && len == 9 && r[4] == 0 && rd32(r + 5) == 1000000U);
    }
    idx = (int)(base + 2); /* remember where this SET_LINK reply lives */
    tick(4000); /* past warmup, batches queued */

    /* e. STOP replies only after both sensors reached STOPPED, so its
     * boundary includes the last batch accepted during an in-flight read. */
    wr32(p, 10);
    base = usb_fake.tx_count;
    feed_request(0x06, 0x11223344U, 9, p, 4);
    tick(1);
    CHECK(find_tx(0x06, base) < 0);
    feed_request(0x06, 0x11223344U, 10, p, 4); /* retry while stopping */
    tick(1);
    CHECK(find_tx(0x06, base) < 0);
    tick(2000);
    probe_acquisition_get_status(PROBE_SPI_ADXL, &st);
    probe_acquisition_get_status(PROBE_SPI_ICM, &st_i);
    CHECK(st.stopped);
    CHECK(st_i.stopped);
    idx = find_tx(0x06, base);
    len = 0;
    {
        uint64_t boundary;
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        CHECK(r && len == 29 && r[4] == 0);
        CHECK(r[5] == 0x01); /* stream id */
        boundary = rd64(r + 9);
        CHECK(boundary == st.last_sequence);
        CHECK(rd64(r + 21) == st_i.last_sequence);
        base = usb_fake.tx_count;
        feed_request(0x06, 0x11223344U, 9, p, 4); /* replay */
        tick(1);
        {
            const uint8_t *r2 = tx_payload(base, &len);
            CHECK(r2 && len == 29 && rd64(r2 + 9) == boundary);
        }
    }
    /* f. SET_LINK accepts the legacy rate codes in standby and echoes them;
     *    unknown rates are rejected. */
    wr32(p, 11); wr32(p + 4, 4000000U);
    feed_request(0x02, 0x11223344U, 10, p, 8);
    tick(1);
    idx = find_tx(0x02, (unsigned)idx + 1); /* skip the reply from step d */
    len = 0;
    {
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        CHECK(r && len == 9 && r[4] == 0 && rd32(r + 5) == 4000000U);
    }
    wr32(p, 12); wr32(p + 4, 9600U);
    feed_request(0x02, 0x11223344U, 11, p, 8);
    tick(1);
    idx = find_tx(0x02, (unsigned)idx + 1);
    len = 0;
    {
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        CHECK(r && len == 5 && r[4] == 4); /* INVALID */
    }

    /* g. FETCH drains queued batches as ADXL/ICM stream frames plus
     *    TIME_INFO, STATUS_NOTIFY and FETCH_DONE. */
    wr32(p, 13);
    feed_request(0x04, 0x11223344U, 12, p, 4);
    tick(1);
    {
        uint16_t l2;
        int i2 = find_tx(0x04, (unsigned)idx + 1);
        const uint8_t *r2 = i2 >= 0 ? tx_payload((unsigned)i2, &l2) : 0;
        if (!r2 || r2[4] != 0) return 3503; /* START must be accepted */ /* TEMP */
    }
    tick(4000); /* warmup plus data */
    {
        probe_acquisition_status sa, si;
        probe_acquisition_get_status(PROBE_SPI_ADXL, &sa);
        probe_acquisition_get_status(PROBE_SPI_ICM, &si);
        if (!sa.running || !si.running) return 3504; /* TEMP */
    }
    reset_usb_fake();
    att_ready = 1;
    wr32(p, 14); wr32(p + 4, 4096U); /* merged batches now reach ~1 KB per frame */
    usb_fake.blocked = 1;
    feed_request(0x05, 0x11223344U, 13, p, 8);
    tick(1);
    CHECK(usb_fake.tx_count == 0);
    CHECK(probe_acquisition_peek(PROBE_SPI_ADXL));
    CHECK(probe_acquisition_peek(PROBE_SPI_ICM));
    usb_fake.blocked = 0;
    run_window(400);
    CHECK(find_tx(0x10, 0) >= 0);           /* STREAM_ADXL */
    CHECK(find_tx(0x11, 0) >= 0);           /* STREAM_ICM */
    CHECK(find_tx(0x20, 0) >= 0);           /* ATTITUDE */
    CHECK(find_tx(0x12, 0) >= 0);           /* TIME_INFO (first window of segment) */
    idx = find_tx(0x13, 0);
    CHECK(idx >= 0);                        /* STATUS_NOTIFY */
    len = 0;
    {
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        /* Queue drops during the unattended run legitimately report WARNING. */
        CHECK(r && len == 16 && (r[0] == 2 || r[0] == 4));
    }
    idx = find_tx(0x10, 0);
    CHECK(idx >= 0);
    len = 0;
    {
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        if (!r) return 3711;
        if (r[29]) return 3700 + r[29] / 100; /* high byte of count */
        if (r[28] < 8) return 3720 + r[28];   /* single reads give 8 samples;
                                               * unattended runs merge to 104 */
        CHECK(len == 32 + (unsigned)r[28] * 9);
        CHECK(rd32(r) == 0x000000ABU);              /* config id */
        CHECK(r[32] == 0xD8 && r[33] == 0xF5 && r[34] == 0x0F);
        CHECK(r[35] == 0x8C && r[36] == 0x0A && r[37] == 0x00);
        CHECK(r[38] == 0x24 && r[39] == 0xC2 && r[40] == 0x00);
        CHECK(rd32(r + 24) == 250U * 65536U);       /* 4000 SPS -> 250 us Q16.16 */
        CHECK((r[30] & 0x01) != 0);                 /* TIME_UNCERTAIN */
    }
    idx = find_tx(0x11, 0);
    CHECK(idx >= 0);
    len = 0;
    {
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        CHECK(r && r[28] >= 16 && len == 32 + (unsigned)r[28] * 13); /* 16B FIFO
                                               * packets become 13-byte samples */
        CHECK(rd32(r + 24) == 125U * 65536U);       /* 8000 SPS -> 125 us */
    }
    idx = find_tx(0x20, 0);
    CHECK(idx >= 0);
    len = 0;
    {
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        CHECK(r && len == 48);
        CHECK(rd32(r) == 0x000000ABU);
        CHECK(rd64(r + 8) == 7 && rd64(r + 16) == 1234567);
        CHECK(rd32(r + 24) == 5000U * 65536U);
        CHECK(r[28] == 1 && r[30] == 1);
        CHECK((int32_t)rd32(r + 32) == 0x10000000);
        CHECK((int32_t)rd32(r + 36) == -0x20000000);
    }
    CHECK(find_tx(0x14, 0) >= 0);
    /* Acquisition continues between asynchronous TX frames. A bounded FETCH
     * may leave new batches for the next window instead of dropping them. */

    /* h. An immediate second window drains remaining/new batches. New segment
     * anchors may require TIME_INFO, but STATUS remains rate-limited. */
    base = usb_fake.tx_count;
    wr32(p, 15); wr32(p + 4, 4096U); /* merged batches now reach ~1 KB per frame */
    feed_request(0x05, 0x11223344U, 14, p, 8);
    run_window(400);
    CHECK(find_tx(0x13, base) < 0);
    CHECK(find_tx(0x14, base) >= 0);

    /* i. STATUS reappears once 500 ms have passed. */
    tick(5500);
    wr32(p, 16); wr32(p + 4, 4096U); /* merged batches now reach ~1 KB per frame */
    feed_request(0x05, 0x11223344U, 15, p, 8);
    run_window(400);
    CHECK(find_tx(0x13, base) >= 0);

    /* j. Fault injection plus nonblocking calibration control. */
    base = usb_fake.tx_count;
    wr32(p, 17); wr32(p + 4, 0x000000ABU);
    p[8] = 2; p[9] = 0x01; p[10] = 4; p[11] = 0; p[12] = 0;
    p[13] = 0x02; p[14] = 5; p[15] = 0x13; p[16] = 0;
    feed_request(0x03, 0xDEADBEEFU, 16, p, 17);
    tick(1);
    CHECK(usb_fake.tx_count == base); /* silently dropped */
    wr32(p, 18);
    feed_request(0x7F, 0x11223344U, 17, p, 4);
    tick(1);
    CHECK(usb_fake.tx_count == base);
    wr32(p, 19); p[4] = 1; p[5] = PROBE_CALIB_TARGET_ICM_ACCEL; p[6] = 0; p[7] = 0;
    feed_request(0x07, 0x11223344U, 18, p, 8);
    tick(1);
    {
        const uint8_t *r = tx_payload(base, &len); /* no reply for the two silent drops */
        CHECK(r && len == 16 && r[4] == PROBE_CALIB_RESULT_IN_PROGRESS);
        CHECK(r[5] == PROBE_CALIB_CAPTURING && r[6] == PROBE_CALIB_TARGET_ICM_ACCEL);
    }
    cal_status.state = PROBE_CALIB_IDLE;
    cal_status.completed_mask = 1;
    cal_status.candidate_valid_flags = PROBE_CALIB_VALID_ICM_ACCEL;
    cal_status.last_result = PROBE_CALIB_RESULT_OK;
    wr32(p, 20); p[4] = 0; p[5] = p[6] = p[7] = 0;
    feed_request(0x07, 0x11223344U, 19, p, 8);
    tick(1);
    {
        const uint8_t *r = tx_payload(base + 1, &len);
        CHECK(r && len == 16 && r[4] == 0 && r[8] == 1 && r[9] == PROBE_CALIB_VALID_ICM_ACCEL);
    }
    wr32(p, 21); p[4] = 1; p[5] = p[6] = p[7] = 0;
    feed_request(0x08, 0x11223344U, 20, p, 8);
    tick(1);
    {
        const uint8_t *r = tx_payload(base + 2, &len);
        CHECK(r && len == 81 && r[4] == 1); /* SAVE is BUSY while measuring */
        CHECK(r[5] == PROBE_CALIB_VERSION && r[6] == PROBE_CALIB_VALID_ICM_ACCEL);
    }
    wr32(p, 22);
    feed_request(0x09, 0x11223344U, 21, p, 4);
    tick(1);
    len = 0;
    {
        const uint8_t *r = tx_payload(base + 3, &len);
        CHECK(r && len == 115);
        CHECK(r[5] & 0x80);                              /* time-uncertain active while measuring */
        CHECK(rd32(r + 95) >= 1);                        /* bad_type counted */
        CHECK(rd32(r + 99) >= 1);                        /* bad_session counted */
        CHECK(r[111] == 1);                              /* last result SAVE BUSY */
    }

    /* k. ADXL EXINT and ICM FIFO-count observations converge: stream frames lose
     *    TIME_UNCERTAIN and TIME_INFO publishes the measured tick rate. */
    {
        unsigned i2;
        uint32_t acc_a = 0;
        for (i2 = 0; i2 < 12000; ++i2) { /* >1 s so TIME_INFO is due again */
            unsigned bus2;
            fake.now += 100;
            acc_a += 100;
            if (acc_a >= 300) { acc_a -= 300; probe_acquisition_irq(0); }
            /* No further ICM watermark IRQs: FIFO polling must map its clock. */
            probe_acquisition_poll();
            probe_proto_poll();
            for (bus2 = 0; bus2 < 2; ++bus2)
                while (probe_acquisition_peek((probe_spi_bus)bus2))
                    probe_acquisition_release((probe_spi_bus)bus2);
        }
        reset_usb_fake();
        uint64_t fresh_cutoff = fake.now; /* projection may lag the anchor */
        tick(120); /* leave a fresh read on each stream (4 ms pacing) */
        wr32(p, 21); wr32(p + 4, 4096U); /* merged batches now reach ~1 KB per frame */
        feed_request(0x05, 0x11223344U, 20, p, 8);
        run_window(400);
        idx = find_tx(0x10, 0);
        CHECK(idx >= 0);
        len = 0;
        {
            const uint8_t *r = tx_payload((unsigned)idx, &len);
            /* tick(120) may catch one or two paced reads. */
            CHECK(r && r[28] >= 8 && len == 32 + (unsigned)r[28] * 9);
            CHECK(!(r[30] & 0x01));            /* TIME_UNCERTAIN cleared */
            CHECK(rd32(r + 24) > 0 && rd32(r + 24) != 250U * 65536U); /* measured, not nominal */        }
        if (find_tx(0x12, 0) < 0) {
            unsigned f;
            for (f = 0; f < usb_fake.tx_count && f < 8; ++f) {
                /* one temp return per captured frame type, offset 4900 */
            }
            return 4900 + usb_fake.tx_count * 32 + (usb_fake.tx_count ? usb_fake.tx[0][4] : 31);
        }
        idx = find_tx(0x12, 0);
        CHECK(idx >= 0);
        len = 0;
        {
            const uint8_t *r = tx_payload((unsigned)idx, &len);
            CHECK(r && len == 40);
            CHECK(r[38] & 0x01);               /* mapping estimated */
            CHECK(rd32(r + 34) > 0 && rd32(r + 34) != 65536U);
            /* The anchor must come from the fresh FIFO-count observations,
             * not the stale IRQ timestamp: it may lag "now" by the
             * estimator's projection lag (~100 ms anchor span) but is
             * ~1 s newer than anything from before the convergence loop. */
            CHECK(rd64(r + 26) > fresh_cutoff - 200000);
            {
                unsigned f, matched = 0;
                uint64_t info_seq = rd64(r + 8), info_anchor = rd64(r + 26);
                for (f = 0; f < usb_fake.tx_count; ++f) if (usb_fake.tx[f][4] == 0x11) {
                    uint16_t stream_len;
                    const uint8_t *stream = tx_payload(f, &stream_len);
                    if (rd64(stream + 8) == info_seq) {
                        CHECK(rd64(stream + 16) == info_anchor);
                        ++matched;
                    }
                }
                CHECK(matched == 1);
            }
        }
        wr32(p, 22);
        feed_request(0x09, 0x11223344U, 21, p, 4);
        tick(1);
        base = usb_fake.tx_count - 1;
        len = 0;
        {
            const uint8_t *r = tx_payload(base, &len);
            CHECK(r && len == 115 && !(r[5] & 0x80)); /* bit7 cleared */
        }
        CHECK(led_last_state == 2); /* healthy measuring drive for the LED */
        CHECK(att_started == 1);    /* attitude branch active while measuring */
    }
    /* l. SAVE_CAL succeeds only after formal STOP and exact retries replay
     *    the 81-byte record without a second flash operation. */
    wr32(p, 23);
    base = usb_fake.tx_count;
    feed_request(0x06, 0x11223344U, 22, p, 4);
    tick(2000);
    CHECK(find_tx(0x06, base) >= 0);
    wr32(p, 24); p[4] = 1; p[5] = p[6] = p[7] = 0;
    base = usb_fake.tx_count;
    feed_request(0x08, 0x11223344U, 23, p, 8);
    tick(1000);
    {
        const uint8_t *r = tx_payload(base, &len);
        CHECK(r && len == 81 && r[4] == 0 && cal_save_count == 1);
    }
    feed_request(0x08, 0x11223344U, 24, p, 8);
    tick(1);
    CHECK(cal_save_count == 1);
    /* A new idle session can save without writing flash while ICM runs.
     * START/SAVE replies may wait for the sensor's safe asynchronous stop. */
    while (probe_acquisition_peek(PROBE_SPI_ADXL)) probe_acquisition_release(PROBE_SPI_ADXL);
    while (probe_acquisition_peek(PROBE_SPI_ICM)) probe_acquisition_release(PROBE_SPI_ICM);
    tick(2000);
    CHECK(probe_acquisition_pose_active());
    wr32(p, 25); p[4] = 1;
    base = usb_fake.tx_count;
    feed_request(0x08, 0x11223344U, 25, p, 8);
    tick(2000);
    idx = find_tx(0x08, base);
    CHECK(idx >= 0 && cal_save_count == 2);
    CHECK(probe_acquisition_pose_active());
    usb_fake.connected = 0;
    tick(2000);
    CHECK(led_last_state == 6 && !probe_acquisition_pose_active() && !att_started);
    usb_fake.connected = 1;
    tick(2000);
    CHECK(!probe_acquisition_pose_active()); /* HELLO required after disconnect */
    wr32(p, 26); wr32(p + 4, 0x55667788U);
    feed_request(0x01, 0, 26, p, 8);
    tick(2000);
    CHECK(probe_acquisition_pose_active() && att_started);
    /* Standby FETCH carries pose only; stopped waveform counters freeze. */
    reset_usb_fake();
    probe_acquisition_get_status(PROBE_SPI_ICM, &st);
    att_ready = 1;
    wr32(p, 27); wr32(p + 4, 1024);
    feed_request(0x05, 0x55667788U, 27, p, 8);
    run_window(400);
    CHECK(find_tx(0x20, 0) >= 0 && find_tx(0x14, 0) >= 0);
    CHECK(find_tx(0x10, 0) < 0 && find_tx(0x11, 0) < 0 && find_tx(0x12, 0) < 0);
    probe_acquisition_get_status(PROBE_SPI_ICM, &st_i);
    CHECK(st_i.samples == st.samples && st_i.last_sequence == st.last_sequence &&
          st_i.queue_drops == st.queue_drops);
    /* Async START retries wait for one completion, then replay its cache. */
    reset_usb_fake();
    wr32(p, 30); wr32(p + 4, 0xCD);
    p[8] = 2; p[9] = 1; p[10] = 4; p[11] = p[12] = 0;
    p[13] = 2; p[14] = 5; p[15] = 0x13; p[16] = 0;
    feed_request(0x03, 0x55667788U, 30, p, 17); tick(1);
    wr32(p, 31);
    base = usb_fake.tx_count;
    feed_request(0x04, 0x55667788U, 31, p, 4); tick(1);
    CHECK(find_tx(0x04, base) < 0 && !att_started);
    feed_request(0x04, 0x55667788U, 32, p, 4); tick(1);
    CHECK(find_tx(0x04, base) < 0);
    tick(4000);
    idx = find_tx(0x04, base);
    CHECK(idx >= 0 && find_tx(0x04, (unsigned)idx + 1) < 0 && att_started);
    probe_acquisition_get_status(PROBE_SPI_ICM, &st);
    CHECK(st.running && st.samples > 0);
    feed_request(0x04, 0x55667788U, 33, p, 4); tick(1);
    CHECK(find_tx(0x04, (unsigned)idx + 1) >= 0);
    probe_acquisition_get_status(PROBE_SPI_ICM, &st_i);
    CHECK(st_i.samples >= st.samples); /* retry did not reset acquisition */

    /* Rebinding a running measurement must never re-label its raw tail. */
    CHECK(probe_acquisition_peek(PROBE_SPI_ICM));
    reset_usb_fake();
    wr32(p, 32); wr32(p + 4, 0xAABBCCDDU);
    feed_request(0x01, 0, 34, p, 8); tick(1);
    att_ready = 0;
    wr32(p, 33); wr32(p + 4, 1024);
    feed_request(0x05, 0xAABBCCDDU, 35, p, 8); run_window(400);
    CHECK(find_tx(0x10, 0) < 0 && find_tx(0x11, 0) < 0);
    tick(3000);
    CHECK(probe_acquisition_pose_active() && !probe_acquisition_peek(PROBE_SPI_ICM));

    /* HELLO also cancels an accepted-but-not-yet-completed START. */
    wr32(p, 34); wr32(p + 4, 0xEF);
    p[8] = 2; p[9] = 1; p[10] = 4; p[11] = p[12] = 0;
    p[13] = 2; p[14] = 5; p[15] = 0x13; p[16] = 0;
    feed_request(0x03, 0xAABBCCDDU, 36, p, 17); tick(1);
    wr32(p, 35);
    base = usb_fake.tx_count;
    feed_request(0x04, 0xAABBCCDDU, 37, p, 4); tick(1);
    CHECK(find_tx(0x04, base) < 0);
    wr32(p, 36); wr32(p + 4, 0x12345678U);
    feed_request(0x01, 0, 38, p, 8); tick(3000);
    CHECK(find_tx(0x04, base) < 0 && probe_acquisition_pose_active());
    CHECK(!probe_acquisition_peek(PROBE_SPI_ICM));

    /* Duplicate pending SAVE performs one flash operation and caches it. */
    wr32(p, 37); p[4] = 1; p[5] = p[6] = p[7] = 0;
    base = usb_fake.tx_count;
    feed_request(0x08, 0x12345678U, 39, p, 8); tick(1);
    CHECK(cal_save_count == 2 && find_tx(0x08, base) < 0);
    feed_request(0x08, 0x12345678U, 40, p, 8); tick(1);
    CHECK(cal_save_count == 2);
    tick(3000);
    idx = find_tx(0x08, base);
    CHECK(idx >= 0 && cal_save_count == 3 && find_tx(0x08, (unsigned)idx + 1) < 0);
    feed_request(0x08, 0x12345678U, 41, p, 8); tick(1);
    CHECK(cal_save_count == 3 && find_tx(0x08, (unsigned)idx + 1) >= 0);

    /* Disconnect cancels pending flash save and requires a fresh HELLO. */
    wr32(p, 38);
    base = usb_fake.tx_count;
    feed_request(0x08, 0x12345678U, 42, p, 8); tick(1);
    CHECK(cal_save_count == 3 && find_tx(0x08, base) < 0);
    usb_fake.connected = 0; tick(3000);
    CHECK(cal_save_count == 3 && !probe_acquisition_pose_active() && !att_started);
    CHECK(find_tx(0x08, base) < 0);
    usb_fake.connected = 1; tick(3000);
    CHECK(cal_save_count == 3 && !probe_acquisition_pose_active());
    /* Optional temperature stays inside FETCH budget and is at most 1 Hz. */
    reset_usb_fake();
    fake.regs[0][0][6] = 7; fake.regs[0][0][7] = 0x5d;
    wr32(p, 50); wr32(p + 4, 0x24681357U);
    feed_request(0x01, 0, 50, p, 8); tick(3000);
    wr32(p, 51); wr32(p + 4, 0xBB);
    p[8] = 2; p[9] = 1; p[10] = 4; p[11] = 0; p[12] = 0;
    p[13] = 2; p[14] = 5; p[15] = 0x13; p[16] = 0;
    feed_request(0x03, 0x24681357U, 51, p, 17); tick(1);
    wr32(p, 52); feed_request(0x04, 0x24681357U, 52, p, 4); tick(15000);
    base = usb_fake.tx_count;
    wr32(p, 53); wr32(p + 4, 34);
    feed_request(0x05, 0x24681357U, 53, p, 8); run_window(300);
    CHECK(find_tx(0x21, base) < 0);
    base = usb_fake.tx_count;
    wr32(p, 54); wr32(p + 4, 35);
    feed_request(0x05, 0x24681357U, 54, p, 8); run_window(300);
    idx = find_tx(0x21, base);
    CHECK(idx >= 0);
    {
        const uint8_t *r = tx_payload((unsigned)idx, &len);
        CHECK(r && len == 16 && rd32(r) == 0xBB && rd64(r + 4) > 0);
        CHECK(r[12] == 0x5d && r[13] == 7 && r[14] == 3 && !r[15]);
        idx = find_tx(0x14, base);
        r = tx_payload((unsigned)idx, &len);
        CHECK(r && rd32(r + 5) == 35 && r[9] == 1);
    }
    base = usb_fake.tx_count;
    wr32(p, 55); wr32(p + 4, 35);
    feed_request(0x05, 0x24681357U, 55, p, 8); run_window(300);
    CHECK(find_tx(0x21, base) < 0);
    return 0;
}
