/**
 * @file probe_proto.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "probe_frame.h"
#include "probe_usb.h"
#include "probe_led.h"
#include "probe_attitude.h"
#include "probe_acquisition.h"
#include "probe_calib.h"
#include "probe_time.h"
#define LOG_TAG "PROTO"
#include "vibra_log.h"

/* Legacy RS485 rate codes. The USB link runs at its own fixed full-speed
 * rate; SET_LINK still accepts these so an unmodified host completes its
 * bringup sequence. */
#define LINK_BPS_BASE 1000000U
#define LINK_BPS_WORK 5000000U
#define LINK_BPS_COMPAT 4000000U
#define WINDOW_DEADLINE_US 20000U
#define STATUS_PERIOD_US 500000U
#define TIME_PERIOD_US 1000000U
#define RESP_TIMEOUT_US 200000U
#define HELLO 0x01
#define SET_LINK 0x02
#define SET_CONFIG 0x03
#define START 0x04
#define FETCH 0x05
#define STOP 0x06
#define CALIBRATE 0x07
#define SAVE_CAL 0x08
#define GET_STATUS 0x09
#define STREAM_ADXL 0x10
#define STREAM_ICM 0x11
#define TIME_INFO 0x12
#define STATUS_NOTIFY 0x13
#define FETCH_DONE 0x14
#define ATTITUDE 0x20
#define TEMPERATURE 0x21 /* Optional ADXL temperature telemetry; legacy frames unchanged. */
#define ATTITUDE_PERIOD_Q16 (5000U * 65536U)

enum {
    RESULT_OK = 0, RESULT_BUSY = 1, RESULT_MALFORMED = 3, RESULT_INVALID = 4,
    RESULT_UNSUPPORTED = 5, RESULT_NOT_READY = 6, RESULT_IN_PROGRESS = 7,
    RESULT_NOT_IMPLEMENTED = 8, RESULT_INTERNAL = 9
};

enum { PROTO_STREAM_ADXL = 0x01, PROTO_STREAM_ICM = 0x02 };

enum { MERGE_HOLD_US = 15000 };
static const uint16_t adxl_odr[5] = {250, 500, 1000, 2000, 4000};
static const uint16_t adxl_range[3] = {10, 20, 40};
static const uint16_t icm_odr[8] = {200, 500, 1000, 2000, 4000, 8000, 16000, 32000};
static const uint16_t icm_accel[4] = {2, 4, 8, 16};
static const uint16_t icm_gyro[4] = {250, 500, 1000, 2000};

typedef struct {
    probe_frame_parser parser;
    probe_acquisition_config config;
    uint8_t frame[PROBE_FRAME_MAX];
    uint8_t payload[PROBE_FRAME_PAYLOAD_MAX];
    uint8_t rx_chunk[256];
    uint32_t session, tx_seq, config_id;
    struct {
        uint8_t type, valid, len;
        uint32_t id;
        uint8_t payload[96]; /* Full mutating reply, including saved calibration. */
    } cache;
    struct {
        uint8_t active;
        uint32_t request_id, budget, used;
        uint16_t frames;
        uint64_t deadline_us, time_sent_us;
        uint8_t icm_sent; /* an ICM frame went out in this window: the
                           * TIME_INFO anchor always pairs with a frame the
                           * host has already seen (merge holds may delay
                           * the first ICM frame past 1 s cadence) */
        uint32_t time_segment, icm_segment;
        uint64_t icm_seq, icm_tick, icm_anchor;
        uint8_t icm_anchored;
        unsigned next_stream;
    } window;
    uint64_t status_sent_us; /* Probe-level 500 ms STATUS cadence */
    uint64_t temperature_sent_us;
    uint32_t bad_types, bad_sessions, busy_rejects;
    uint8_t result;
    uint8_t measuring;
    uint8_t stop_pending, pending_type, discard_raw;
    uint32_t pending_id;
    uint32_t stop_request_id;
} proto_context;

static proto_context proto;

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }
static void put64(uint8_t *p, uint64_t v) { put32(p, (uint32_t)v); put32(p + 4, (uint32_t)(v >> 32)); }
static void put_float(uint8_t *p, float v) { uint32_t bits; memcpy(&bits, &v, sizeof(bits)); put32(p, bits); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

/* ADXL FIFO words arrive MSB first with marker bits in the low nibble of the
 * third byte. The wire format is a packed little-endian 20-bit value. */
static void pack_adxl_axis(uint8_t *out, const uint8_t *fifo)
{
    uint32_t value = ((uint32_t)fifo[0] << 12) |
                     ((uint32_t)fifo[1] << 4) |
                     ((uint32_t)fifo[2] >> 4);
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
    out[2] = (uint8_t)((value >> 16) & 0x0FU);
}

static int send_frame(uint8_t type, const uint8_t *payload, unsigned len)
{
    unsigned total = probe_frame_build(proto.frame, type, proto.session, proto.tx_seq++, payload, len);
    return total && probe_usb_send(proto.frame, total, RESP_TIMEOUT_US);
}

/* Mutating requests cache their full reply; identical retries replay it. */
static void cache_store(uint8_t type, uint32_t id, const uint8_t *payload, unsigned len)
{
    if (len > sizeof(proto.cache.payload)) return;
    proto.cache.type = type;
    proto.cache.id = id;
    proto.cache.len = (uint8_t)len;
    memcpy(proto.cache.payload, payload, len);
    proto.cache.valid = 1;
}

static void send_result(uint8_t type, uint32_t request_id, uint8_t result)
{
    put32(proto.payload, request_id);
    proto.payload[4] = result;
    proto.result = result;
    send_frame(type, proto.payload, 5);
}

static int send_stop_result(uint32_t request_id)
{
    probe_acquisition_status a, i;
    uint8_t *p = proto.payload;
    probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
    probe_acquisition_get_status(PROBE_SPI_ICM, &i);
    put32(p, request_id);
    p[4] = RESULT_OK;
    p[5] = PROTO_STREAM_ADXL; p[6] = p[7] = p[8] = 0;
    put64(p + 9, a.last_sequence);
    p[17] = PROTO_STREAM_ICM; p[18] = p[19] = p[20] = 0;
    put64(p + 21, i.last_sequence);
    if (!send_frame(STOP, p, 29)) return 0;
    proto.result = RESULT_OK;
    cache_store(STOP, request_id, p, 29);
    return 1;
}

static void stop_service(void)
{
    probe_acquisition_status a, i;
    if (!proto.stop_pending || proto.window.active || probe_usb_tx_busy()) return;
    probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
    probe_acquisition_get_status(PROBE_SPI_ICM, &i);
    if (!a.stopped || !i.stopped) return;
    if (send_stop_result(proto.stop_request_id)) proto.stop_pending = 0;
}

static uint32_t period_q16(unsigned odr_hz)
{
    return (uint32_t)(((uint64_t)1000000 << 16) / odr_hz);
}

static void fault_bits(probe_acquisition_status *a, probe_acquisition_status *i,
                       int measuring, uint16_t *bits)
{
    static const uint8_t map[8] = {0, 0, 5, 3, 4, 5, 2, 5}; /* acquisition fault -> bit index */
    *bits = 0;
    if (measuring && (!a->time_valid || !i->time_valid)) *bits |= 1U << 7;
    /* last_fault latches for diagnostics; the wire state reports active
     * conditions only, so a rebuilt stream returns to green. */
    if (!a->running && a->last_fault && a->last_fault < 8) {
        if (a->last_fault == 2) *bits |= 1U << 0; /* identity failure kills the main channel */
        else *bits |= 1U << map[a->last_fault];
    }
    if (!i->running && i->last_fault && i->last_fault < 8) {
        if (i->last_fault == 2) *bits |= 1U << 1;
        else *bits |= 1U << map[i->last_fault];
    }
}

static uint8_t work_state(uint16_t fault)
{
    probe_calib_status cal;
    if (!proto.session) return 0;
    probe_calib_get_status(&cal);
    if (cal.state == PROBE_CALIB_CAPTURING) return 3;
    if (!proto.measuring) return 1;
    if (!fault) return 2;
    return (fault & 0x0003U) ? 5 : 4; /* main channel failure outranks warnings */
}

static void send_status(uint64_t now)
{
    probe_acquisition_status a, i;
    uint16_t bits;
    uint8_t *p = proto.payload;
    probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
    probe_acquisition_get_status(PROBE_SPI_ICM, &i);
    fault_bits(&a, &i, proto.measuring, &bits);
    p[0] = work_state(bits);
    put16(p + 1, bits);
    p[3] = p[0] == 0 ? 0 : p[0]; /* LED id mirrors the state enum */
    put16(p + 4, 0);             /* constant-on states have no period */
    put32(p + 6, 0);             /* phase not tracked in this version */
    put32(p + 10, (uint32_t)(now / 1000));
    put16(p + 14, 0);
    send_frame(STATUS_NOTIFY, p, 16);
}

static void send_time_info(void)
{
    uint8_t *p = proto.payload;
    probe_acquisition_status i;
    probe_acquisition_get_status(PROBE_SPI_ICM, &i);
    put32(p, proto.config_id);
    put32(p + 4, i.segment);
    put64(p + 8, proto.window.icm_seq);
    p[16] = 1;                            /* one mapping entry */
    p[17] = 2;                            /* source: ICM internal timestamp */
    put64(p + 18, proto.window.icm_tick);
    put64(p + 26, proto.window.icm_anchor);
    put32(p + 34, i.tick_valid ? i.tick_period_q16 : 65536U); /* MCU us per tick */
    put16(p + 38, i.tick_valid ? 0x0001U : 0x0000U);          /* mapping estimated */
    send_frame(TIME_INFO, p, 40);
}

static int try_send_stream(unsigned bus)
{
    probe_spi_bus target = bus ? PROBE_SPI_ICM : PROBE_SPI_ADXL;
    const probe_raw_batch *batch = probe_acquisition_peek(target);
    unsigned per_sample = bus ? 13U : 9U;
    unsigned samples, total, i, out = 0;
    uint8_t *p;
    if (!batch || proto.discard_raw) return 0;
    samples = batch->samples;
    /* Hold an under-filled head slot so paced reads merge into full wire
     * frames. Board round 19: halving the read intervals fixed the FIFO
     * starvation but let the host pump pull every small read as its own
     * frame (~1060 fps), and the USB link dropped twice at that packet
     * rate. Send early under pressure: more slots queued behind it, a
     * pending STOP, a closing window, or the hold elapsed. */
    if (samples < (bus ? PROBE_ICM_BATCH_SAMPLES : PROBE_ADXL_BATCH_SAMPLES)) {
        probe_acquisition_status st;
        probe_acquisition_get_status(target, &st);
        if (st.queued_batches <= 1U && !proto.stop_pending &&
            probe_time_us() + 2000U < proto.window.deadline_us &&
            probe_time_us() - batch->read_started_us < MERGE_HOLD_US)
            return 0;
    }
    total = 19U + 32U + samples * per_sample;
    if (proto.window.used + total > proto.window.budget) return 0;
    p = proto.payload;
    put32(p, proto.config_id);
    put32(p + 4, batch->segment);
    put64(p + 8, batch->first_sequence);
    /* Estimated first-sample time once the timeline converged; before that
     * the delivery anchor is the only honest value and the batch stays
     * flagged TIME_UNCERTAIN. */
    put64(p + 16, (batch->flags & PROBE_BATCH_TIME_UNCERTAIN) ?
                  batch->read_started_us : batch->first_sample_us);
    put32(p + 24, batch->period_q16 ? batch->period_q16 :
                  period_q16(bus ? proto.config.icm.odr_hz : proto.config.adxl.odr_hz));
    put16(p + 28, (uint16_t)samples);
    put16(p + 30, (uint16_t)((batch->flags & PROBE_BATCH_TIME_UNCERTAIN ? 0x0001U : 0U) |
                             (batch->flags & PROBE_BATCH_SEGMENT_START ? 0x0002U : 0U) |
                             (batch->flags & PROBE_BATCH_NEAR_FULL_SCALE ? 0x0004U : 0U)));
    if (bus) {
        icm42688_fifo_sample sample;
        for (i = 0; i + ICM42688_FIFO_PACKET_BYTES <= batch->size; i += ICM42688_FIFO_PACKET_BYTES) {
            if (!icm42688_decode_fifo(batch->data + i, ICM42688_FIFO_PACKET_BYTES, &sample))
                memset(p + 32 + out, 0, 13);
            else {
                put16(p + 32 + out, (uint16_t)sample.accel[0]);
                put16(p + 34 + out, (uint16_t)sample.accel[1]);
                put16(p + 36 + out, (uint16_t)sample.accel[2]);
                put16(p + 38 + out, (uint16_t)sample.gyro[0]);
                put16(p + 40 + out, (uint16_t)sample.gyro[1]);
                put16(p + 42 + out, (uint16_t)sample.gyro[2]);
                p[44 + out] = (uint8_t)sample.temperature;
            }
            out += 13;
        }
        proto.window.icm_seq = batch->first_sequence;
        proto.window.icm_tick = batch->first_sensor_tick;
        proto.window.icm_anchor = batch->first_sample_us; /* same sample as icm_tick/seq */
        proto.window.icm_anchored = 1;
        proto.window.icm_segment = batch->segment;
    } else {
        for (i = 0; i < samples; ++i) {
            unsigned axis;
            for (axis = 0; axis < 3; ++axis)
                pack_adxl_axis(p + 32 + i * 9U + axis * 3U,
                               batch->data + i * 9U + axis * 3U);
        }
    }
    if (!send_frame(bus ? STREAM_ICM : STREAM_ADXL, p, 32 + samples * per_sample))
        return 0;
    if (bus) proto.window.icm_sent = 1;
    probe_calib_ingest(target, batch, probe_time_us());
    probe_acquisition_release(target);
    proto.window.used += total;
    ++proto.window.frames;
    return 1;
}

static int try_send_attitude(void)
{
    const probe_attitude_sample *sample = probe_attitude_peek();
    probe_acquisition_status i;
    uint8_t *p = proto.payload;
    unsigned axis;
    const unsigned total = 19U + 48U;
    if (!sample || proto.window.used + total > proto.window.budget) return 0;
    probe_acquisition_get_status(PROBE_SPI_ICM, &i);
    put32(p, proto.config_id);
    put32(p + 4, i.segment);
    put64(p + 8, sample->sequence);
    put64(p + 16, sample->time_us);
    put32(p + 24, ATTITUDE_PERIOD_Q16);
    put16(p + 28, 1);
    put16(p + 30, sample->flags);
    for (axis = 0; axis < 4; ++axis) put32(p + 32 + axis * 4U, (uint32_t)sample->quat_q30[axis]);
    if (!send_frame(ATTITUDE, p, 48)) return 0;
    probe_attitude_release();
    proto.window.used += total;
    ++proto.window.frames;
    return 1;
}

static int try_send_window_item(unsigned stream)
{
    if (stream < 2U) return try_send_stream(stream);
    return try_send_attitude();
}

static void send_calibrate_status(uint32_t request_id, uint8_t result, int cache_reply)
{
    probe_calib_status cal;
    uint8_t *p = proto.payload;
    probe_calib_get_status(&cal);
    put32(p, request_id);
    p[4] = result;
    p[5] = cal.state;
    p[6] = cal.target;
    p[7] = cal.face;
    p[8] = cal.completed_mask;
    p[9] = cal.candidate_valid_flags;
    p[10] = cal.dirty;
    p[11] = cal.last_result;
    put32(p + 12, cal.sample_count);
    proto.result = result;
    if (cache_reply) cache_store(CALIBRATE, request_id, p, 16);
    send_frame(CALIBRATE, p, 16);
}

static void send_calibration_record(uint32_t request_id, uint8_t result, int cache_reply)
{
    probe_calib_record record;
    uint8_t *p = proto.payload;
    unsigned axis;
    probe_calib_get_candidate(&record);
    put32(p, request_id);
    p[4] = result;
    p[5] = record.version;
    p[6] = record.valid_flags;
    p[7] = p[8] = 0;
    put32(p + 9, record.generation);
    put32(p + 13, record.created_ms);
    for (axis = 0; axis < 3; ++axis) put_float(p + 17 + axis * 4U, record.accel_bias_g[axis]);
    for (axis = 0; axis < 3; ++axis) put_float(p + 29 + axis * 4U, record.accel_scale[axis]);
    for (axis = 0; axis < 3; ++axis) put_float(p + 41 + axis * 4U, record.icm_accel_bias_g[axis]);
    for (axis = 0; axis < 3; ++axis) put_float(p + 53 + axis * 4U, record.icm_accel_scale[axis]);
    for (axis = 0; axis < 3; ++axis) put_float(p + 65 + axis * 4U, record.gyro_bias_dps[axis]);
    put_float(p + 77, record.temp_c);
    proto.result = result;
    if (cache_reply) cache_store(SAVE_CAL, request_id, p, 81);
    send_frame(SAVE_CAL, p, 81);
}

static void finish_window(uint64_t now)
{
    probe_acquisition_status a, i;
    uint8_t *p = proto.payload;
    probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
    probe_acquisition_get_status(PROBE_SPI_ICM, &i);
    put32(p, proto.window.request_id);
    p[4] = RESULT_OK;
    put32(p + 5, proto.window.used);
    put16(p + 9, proto.window.frames);
    put16(p + 11, a.queued_batches);
    put32(p + 13, a.queued_bytes);
    put16(p + 17, i.queued_batches);
    put32(p + 19, i.queued_bytes);
    p[23] = work_state(0);
    put16(p + 24, 0);
    p[26] = 0;
    if (send_frame(FETCH_DONE, p, 27)) proto.window.active = 0;
    (void)now;
}

static void window_service(uint64_t now)
{
    unsigned attempt;
    if (!proto.window.active || probe_usb_tx_busy()) return;
    if (now >= proto.window.deadline_us) { finish_window(now); return; }
    if (now - proto.status_sent_us >= STATUS_PERIOD_US) {
        send_status(now);
        proto.status_sent_us = now;
        return;
    }
    if (now - proto.temperature_sent_us >= 1000000U &&
        proto.window.used + 35U <= proto.window.budget) {
        probe_temperature temp;
        probe_acquisition_get_temperature(&temp);
        if (temp.valid) {
            uint8_t *p = proto.payload;
            put32(p, proto.config_id);
            put64(p + 4, temp.time_us);
            put16(p + 12, temp.raw);
            p[14] = (uint8_t)(1U | (temp.live ? 2U : 0U)); p[15] = 0;
            if (send_frame(TEMPERATURE, p, 16)) {
                proto.temperature_sent_us = now;
                proto.window.used += 35U;
                ++proto.window.frames;
                return;
            }
        }
    }
    /* Burst: emit frames back-to-back, waiting only for each frame's own
     * packets to clear the wire. With ~1 KB frames the old one-frame-per-
     * loop-poll rule turned the loop period into a ~370 fps ceiling that sat
     * right on 4k/8k demand (board round 16) while the wire itself was ~75%
     * idle. */
    for (;;) {
        unsigned progressed = 0;
        for (attempt = 0; attempt < 3U; ++attempt) {
            unsigned stream = (proto.window.next_stream + attempt) % 3U;
            if (try_send_window_item(stream)) {
                /* Re-serve a raw stream while it holds the larger backlog: at
                 * 8k ODR strict turns capped ICM at ~1/3 of the frame rate and
                 * kept its queue saturated, which churned segments and shrank
                 * batches. Empty partners rotate on, so slow-ODR behaviour
                 * stays round-robin. */
                if (stream < 2U) {
                    probe_acquisition_status mine, other;
                    probe_acquisition_get_status(stream == 0U ? PROBE_SPI_ADXL : PROBE_SPI_ICM, &mine);
                    probe_acquisition_get_status(stream == 0U ? PROBE_SPI_ICM : PROBE_SPI_ADXL, &other);
                    if (mine.queued_bytes >= other.queued_bytes)
                        proto.window.next_stream = stream;
                    else
                        proto.window.next_stream = (stream + 1U) % 3U;
                } else {
                    proto.window.next_stream = (stream + 1U) % 3U;
                }
                progressed = 1;
                break;
            }
        }
        if (!progressed) break;
        /* USB hosts pace IN packets (this one ~1 packet/ms): a 1 KB frame
         * holds the wire for ~16 ms. RS485 DMA released the loop after pure
         * wire time; waiting idle here again starved the sensor FIFOs (board
         * round 17: ICM overflowed on every large-frame burst), so each
         * packet-completion gap now services the acquisition state machines. */
        while (probe_usb_tx_busy()) {
            probe_usb_poll();      /* pure wire time, one packet at a time */
            probe_acquisition_poll();
            probe_attitude_poll();
        }
        now = probe_time_us();
        if (now >= proto.window.deadline_us) { finish_window(now); return; }
        if (now - proto.status_sent_us >= STATUS_PERIOD_US) return;
    }
    /* The ICM clock anchor exists only once its first batch went out; the
     * mapping rides after it, once per segment and then every second. */
    if (proto.window.icm_sent && proto.window.icm_anchored &&
        (proto.window.icm_segment != proto.window.time_segment ||
         now - proto.window.time_sent_us >= TIME_PERIOD_US)) {
        send_time_info();
        proto.window.time_segment = proto.window.icm_segment;
        proto.window.time_sent_us = now;
        return;
    }
    /* Finish only when nothing is queued anywhere: a held slot keeps
     * merging reads while the window stays open, bounded by the deadline
     * checked at the top. */
    if (!probe_acquisition_peek(PROBE_SPI_ADXL) && !probe_acquisition_peek(PROBE_SPI_ICM) &&
        !probe_attitude_peek())
        finish_window(now);
}

static void respond_hello(uint32_t request_id)
{
    uint8_t *p = proto.payload;
    probe_acquisition_status a, i;
    probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
    probe_acquisition_get_status(PROBE_SPI_ICM, &i);
    put32(p, request_id);
    p[4] = RESULT_OK;
    put16(p + 5, 0x0001U); /* probe type */
    p[7] = 1;              /* hardware revision */
    put32(p + 8, 0x00000001U); /* firmware 0.1 */
    put32(p + 12, (uint32_t)(probe_time_us() / 1000));
    put16(p + 16, 0x0007U); /* legacy RS485 rate codes; link itself is USB CDC */
    put16(p + 18, (uint16_t)((a.running || a.samples ? 0x0001U : 0U) |
                             (i.running || i.samples ? 0x0002U : 0U)));
    put32(p + 20, proto.config_id);
    p[24] = work_state(0);
    put32(p + 25, 0);
    proto.result = RESULT_OK;
    send_frame(HELLO, p, 29);
}

static uint8_t configure_stream(const uint8_t *entry, probe_acquisition_config *out)
{
    if (entry[0] == PROTO_STREAM_ADXL) {
        if (entry[1] >= 5 || entry[2] >= 3) return RESULT_INVALID;
        if (entry[3]) return RESULT_UNSUPPORTED;
        out->adxl.odr_hz = adxl_odr[entry[1]];
        out->adxl.range_g = adxl_range[entry[2]];
        out->adxl.hpf = 0;
        return RESULT_OK;
    }
    if (entry[0] == PROTO_STREAM_ICM) {
        if (entry[1] >= 8 || (entry[2] & 0x0FU) >= 4 || (entry[2] >> 4) >= 4) return RESULT_INVALID;
        if (entry[3]) return RESULT_UNSUPPORTED;
        out->icm.odr_hz = icm_odr[entry[1]];
        out->icm.accel_g = icm_accel[entry[2] & 0x0FU];
        out->icm.gyro_dps = icm_gyro[entry[2] >> 4];
        return RESULT_OK;
    }
    return RESULT_INVALID;
}

/* Pose hardware is private to standby. START and flash save wait until its
 * asynchronous power-down finishes; no fake measurement state or BUSY retry. */
static void pending_service(void)
{
    probe_acquisition_status a, i;
    uint8_t type = proto.pending_type, result;
    uint32_t id = proto.pending_id;
    if (!type || proto.window.active || probe_usb_tx_busy() ||
        probe_acquisition_pose_active()) return;
    probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
    probe_acquisition_get_status(PROBE_SPI_ICM, &i);
    if (!a.stopped || !i.stopped) return;
    proto.pending_type = 0;
    if (type == START) {
        result = probe_acquisition_start(&proto.config) ? RESULT_OK : RESULT_INTERNAL;
        if (result == RESULT_OK) {
            proto.measuring = 1;
            probe_attitude_start(&proto.config.icm);
        }
        put32(proto.payload, id); proto.payload[4] = result;
        proto.result = result;
        cache_store(START, id, proto.payload, 5);
        send_frame(START, proto.payload, 5);
    } else {
        result = probe_calib_save_candidate((uint32_t)(probe_time_us() / 1000U));
        send_calibration_record(id, result, 1);
    }
}

static void standby_service(void)
{
    static const icm42688_config pose = {200, 16, 500};
    probe_acquisition_status a, i;
    probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
    probe_acquisition_get_status(PROBE_SPI_ICM, &i);
    if (proto.discard_raw) {
        while (probe_acquisition_peek(PROBE_SPI_ADXL)) probe_acquisition_release(PROBE_SPI_ADXL);
        while (probe_acquisition_peek(PROBE_SPI_ICM)) probe_acquisition_release(PROBE_SPI_ICM);
        if (a.stopped && i.stopped && !probe_acquisition_pose_active()) proto.discard_raw = 0;
        else return;
    }
    if (!proto.session || proto.measuring || proto.pending_type ||
        proto.stop_pending || proto.window.active || !a.stopped || !i.stopped ||
        probe_acquisition_pose_active() || !probe_usb_connected()) return;
    if (probe_acquisition_pose_start(&pose)) {
        probe_attitude_start(&pose);
        probe_attitude_latest_only(1);
    }
}

static void dispatch(const probe_frame_parser *f)
{
    uint64_t now = probe_time_us();
    uint32_t request_id;
    if (f->payload_len < 4) {
        if (f->type != FETCH) { send_result(f->type, 0, RESULT_MALFORMED); return; }
        request_id = 0; /* unreachable: FETCH requires 8 bytes */
    }
    request_id = get32(f->payload);
    if (f->type != HELLO &&
        (f->session != proto.session || !proto.session)) {
        ++proto.bad_sessions; /* structurally valid, wrong session: silent */
        return;
    }
    if (f->type != HELLO && proto.cache.valid &&
        proto.cache.type == f->type && proto.cache.id == request_id) {
        send_frame(f->type, proto.cache.payload, proto.cache.len);
        return;
    }
    if (proto.pending_type && f->type != HELLO && f->type != FETCH && f->type != GET_STATUS) {
        if (f->type == proto.pending_type && request_id == proto.pending_id) return;
        send_result(f->type, request_id, RESULT_BUSY);
        return;
    }
    if (proto.stop_pending && f->type != STOP && f->type != FETCH &&
        f->type != GET_STATUS && f->type != HELLO) {
        ++proto.busy_rejects;
        send_result(f->type, request_id, RESULT_BUSY);
        return;
    }
    switch (f->type) {
    case HELLO: {
        uint32_t proposed;
        if (f->payload_len != 8) { send_result(HELLO, request_id, RESULT_MALFORMED); return; }
        proposed = get32(f->payload + 4);
        if (!proposed) { send_result(HELLO, request_id, RESULT_INVALID); return; }
        if (proto.window.active) proto.window.active = 0;
        probe_acquisition_stop();
        probe_attitude_stop();
        proto.measuring = 0;
        proto.pending_type = 0;
        proto.discard_raw = 1; /* Never re-label old raw batches with a new session. */
        probe_attitude_init();
        probe_calib_cancel();
        proto.stop_pending = 0;
        proto.config_id = 0;
        proto.session = proposed;
        proto.cache.valid = 0;
        proto.result = RESULT_OK;
        respond_hello(request_id);
        return;
    }
    case SET_LINK: {
        uint32_t bps;
        if (f->payload_len != 8) { send_result(SET_LINK, request_id, RESULT_MALFORMED); return; }
        bps = get32(f->payload + 4);
        if (bps != LINK_BPS_BASE && bps != LINK_BPS_WORK && bps != LINK_BPS_COMPAT) {
            send_result(SET_LINK, request_id, RESULT_INVALID);
            return;
        }
        /* The USB link has no configurable rate; the accepted code is echoed
         * so an unmodified host completes its bringup unchanged. */
        put32(proto.payload, request_id);
        proto.payload[4] = RESULT_OK;
        put32(proto.payload + 5, bps);
        proto.result = RESULT_OK;
        cache_store(f->type, request_id, proto.payload, 9);
        send_frame(SET_LINK, proto.payload, 9);
        return;
    }
    case SET_CONFIG: {
        uint8_t r;
        if (proto.measuring || proto.window.active) {
            ++proto.busy_rejects;
            send_result(SET_CONFIG, request_id, RESULT_BUSY);
            return;
        }
        if (f->payload_len != 17) { send_result(SET_CONFIG, request_id, RESULT_MALFORMED); return; }
        if (!get32(f->payload + 4) || f->payload[8] != 2) {
            send_result(SET_CONFIG, request_id, RESULT_INVALID);
            return;
        }
        r = configure_stream(f->payload + 9, &proto.config);
        if (r == RESULT_OK) r = configure_stream(f->payload + 13, &proto.config);
        if (r != RESULT_OK) {
            put32(proto.payload, request_id);
            proto.payload[4] = r;
            proto.result = r;
            if (r == RESULT_UNSUPPORTED) {
                proto.payload[5] = f->payload[9]; /* name the offending stream */
                send_frame(SET_CONFIG, proto.payload, 6);
            } else {
                send_frame(SET_CONFIG, proto.payload, 5);
            }
            return;
        }
        proto.config_id = get32(f->payload + 4);
        proto.payload[5] = 2;
        memcpy(proto.payload + 6, f->payload + 9, 8); /* echo effective entries */
        put32(proto.payload, request_id);
        proto.payload[4] = RESULT_OK;
        proto.result = RESULT_OK;
        cache_store(f->type, request_id, proto.payload, 14);
        send_frame(SET_CONFIG, proto.payload, 14);
        return;
    }
    case START: {
        if (f->payload_len != 4) { send_result(START, request_id, RESULT_MALFORMED); return; }
        if (proto.measuring) {
            ++proto.busy_rejects;
            send_result(START, request_id, RESULT_BUSY);
            return;
        }
        if (!proto.config_id) { send_result(START, request_id, RESULT_NOT_READY); return; }
        proto.pending_type = START;
        proto.pending_id = request_id;
        if (probe_acquisition_pose_active()) {
            probe_acquisition_stop();
            probe_attitude_stop();
        }
        pending_service();
        return;
    }
    case STOP: {
        if (f->payload_len != 4) { send_result(STOP, request_id, RESULT_MALFORMED); return; }
        if (proto.stop_pending) {
            if (request_id != proto.stop_request_id) {
                ++proto.busy_rejects;
                send_result(STOP, request_id, RESULT_BUSY);
            }
            return;
        }
        if (proto.measuring) {
            probe_acquisition_stop();
            probe_attitude_stop();
            proto.measuring = 0;
            proto.stop_pending = 1;
            proto.stop_request_id = request_id;
            return;
        }
        send_stop_result(request_id);
        return;
    }
    case FETCH: {
        if (f->payload_len != 8) { send_result(FETCH, request_id, RESULT_MALFORMED); return; }
        if (proto.window.active) {
            ++proto.busy_rejects;
            send_result(FETCH, request_id, RESULT_BUSY);
            return;
        }
        proto.window.active = 1;
        proto.window.request_id = request_id;
        proto.window.budget = get32(f->payload + 4);
        proto.window.used = 0;
        proto.window.frames = 0;
        proto.window.deadline_us = now + WINDOW_DEADLINE_US;
        proto.window.next_stream = 0;
        proto.window.icm_sent = 0;
        proto.result = RESULT_OK;
        window_service(now); /* stream immediately out of this poll */
        return;
    }
    case CALIBRATE: {
        uint8_t action, target, face, result;
        if (f->payload_len != 8) { send_result(CALIBRATE, request_id, RESULT_MALFORMED); return; }
        action = f->payload[4]; target = f->payload[5]; face = f->payload[6];
        if (f->payload[7]) { send_result(CALIBRATE, request_id, RESULT_INVALID); return; }
        if (action == 0) { send_calibrate_status(request_id, RESULT_OK, 0); return; }
        if (action == 1) {
            if (!proto.measuring) result = RESULT_NOT_READY;
            else result = probe_calib_begin(target, face, &proto.config, now);
        } else if (action == 2) {
            probe_calib_cancel(); result = RESULT_OK;
        } else if (action == 3 && (target == PROBE_CALIB_TARGET_ADXL_ACCEL ||
                                  target == PROBE_CALIB_TARGET_ICM_ACCEL)) {
            probe_calib_status cal;
            probe_calib_get_status(&cal);
            if (cal.state == PROBE_CALIB_CAPTURING) result = RESULT_BUSY;
            else { probe_calib_reset_progress(target); result = RESULT_OK; }
        } else result = RESULT_INVALID;
        send_calibrate_status(request_id, result, 1);
        return;
    }
    case SAVE_CAL: {
        probe_acquisition_status a, i;
        uint8_t action, result;
        if (f->payload_len != 8) { send_result(SAVE_CAL, request_id, RESULT_MALFORMED); return; }
        action = f->payload[4];
        if (f->payload[5] || f->payload[6] || f->payload[7] || action > 1U) {
            send_result(SAVE_CAL, request_id, RESULT_INVALID); return;
        }
        if (action == 0) { send_calibration_record(request_id, RESULT_OK, 0); return; }
        probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
        probe_acquisition_get_status(PROBE_SPI_ICM, &i);
        if (proto.measuring || !a.stopped || !i.stopped) result = RESULT_BUSY;
        else if (probe_acquisition_pose_active()) {
            proto.pending_type = SAVE_CAL;
            proto.pending_id = request_id;
            probe_acquisition_stop();
            probe_attitude_stop();
            return;
        } else result = probe_calib_save_candidate((uint32_t)(now / 1000U));
        send_calibration_record(request_id, result, 1);
        return;
    }
    case GET_STATUS: {
        probe_acquisition_status a, i;
        uint16_t bits;
        uint8_t *p = proto.payload;
        if (f->payload_len != 4) { send_result(GET_STATUS, request_id, RESULT_MALFORMED); return; }
        probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
        probe_acquisition_get_status(PROBE_SPI_ICM, &i);
        fault_bits(&a, &i, proto.measuring, &bits);
        put32(p, (uint32_t)(now / 1000));
        p[4] = work_state(bits);
        put16(p + 5, bits);
        put32(p + 7, proto.config_id);
        p[11] = PROTO_STREAM_ADXL; p[12] = p[13] = p[14] = 0;
        put64(p + 15, a.samples);
        put32(p + 23, a.segment);
        put32(p + 27, a.unknown_gaps);
        put16(p + 31, a.queued_batches);
        put32(p + 33, a.queued_bytes);
        put32(p + 37, a.spi_errors);
        put32(p + 41, a.overflows);
        put32(p + 45, a.format_errors);
        put32(p + 49, a.queue_drops);
        p[53] = PROTO_STREAM_ICM; p[54] = p[55] = p[56] = 0;
        put64(p + 57, i.samples);
        put32(p + 65, i.segment);
        put32(p + 69, i.unknown_gaps);
        put16(p + 73, i.queued_batches);
        put32(p + 75, i.queued_bytes);
        put32(p + 79, i.spi_errors);
        put32(p + 83, i.overflows);
        put32(p + 87, i.format_errors);
        put32(p + 91, i.queue_drops);
        put32(p + 95, proto.bad_types);
        put32(p + 99, proto.bad_sessions);
        put32(p + 103, proto.busy_rejects);
        put32(p + 107, proto.parser.crc_errors);
        p[111] = proto.result;
        p[112] = p[113] = p[114] = 0;
        proto.result = RESULT_OK;
        send_frame(GET_STATUS, p, 115);
        return;
    }
    default:
        ++proto.bad_types; /* valid frame, no such message on this wire */
        return;
    }
}

void probe_proto_init(void)
{
    memset(&proto, 0, sizeof(proto));
    probe_usb_init(); /* Enumerate and arm the CDC endpoints before requests. */
    probe_frame_parser_init(&proto.parser, probe_time_us());
    LOG_I("USB CDC protocol link ready");
}

void probe_proto_poll(void)
{
    uint64_t now = probe_time_us();
    unsigned n;
    if (!probe_usb_connected() && proto.session) {
        proto.session = proto.config_id = 0;
        proto.measuring = proto.stop_pending = proto.pending_type = proto.window.active = 0;
        proto.cache.valid = 0;
        proto.discard_raw = 1;
        probe_acquisition_stop();
        probe_attitude_stop();
        probe_attitude_init();
        probe_calib_cancel();
        probe_frame_parser_init(&proto.parser, now);
    }
    n = probe_usb_read(proto.rx_chunk, sizeof(proto.rx_chunk));
    if (n && probe_frame_parse(&proto.parser, proto.rx_chunk, n, now))
        dispatch(&proto.parser);
    probe_usb_poll();
    probe_calib_poll(now);
    stop_service();
    standby_service();
    pending_service();
    window_service(now);
    {
        /* Single source of truth: the same state value feeds STATUS_NOTIFY,
         * GET_STATUS and the LED. State 6 (no USB host) is LED-local: frames
         * only flow while the host is up, so the wire never carries it. */
        probe_acquisition_status a, i;
        uint16_t bits;
        probe_acquisition_get_status(PROBE_SPI_ADXL, &a);
        probe_acquisition_get_status(PROBE_SPI_ICM, &i);
        fault_bits(&a, &i, proto.measuring, &bits);
        probe_led_update(probe_usb_connected() ? work_state(bits) : 6);
    }
}
