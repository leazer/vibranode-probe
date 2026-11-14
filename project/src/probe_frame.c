/**
 * @file probe_frame.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "probe_frame.h"

#define CRC32C_REFLECTED 0x82F63B78U
/* A frame must arrive contiguously; longer gaps reset the parser. */
#define FRAME_GAP_TIMEOUT_US 50000U

enum { STATE_SCAN, STATE_FIXED, STATE_PAYLOAD };

static uint32_t crc_span(uint32_t crc, const uint8_t *data, unsigned size)
{
    while (size--) {
        int bit;
        crc ^= *data++;
        for (bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (CRC32C_REFLECTED & (0U - (crc & 1)));
    }
    return crc;
}

uint32_t probe_frame_crc32c(const uint8_t *data, unsigned size)
{
    return crc_span(0xFFFFFFFFU, data, size) ^ 0xFFFFFFFFU;
}

static void put_le32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
    out[2] = (uint8_t)(value >> 16);
    out[3] = (uint8_t)(value >> 24);
}

static uint32_t get_le32(const uint8_t *in)
{
    return (uint32_t)in[0] | ((uint32_t)in[1] << 8) |
           ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
}

unsigned probe_frame_build(uint8_t *out, uint8_t type, uint32_t session, uint32_t seq,
                           const uint8_t *payload, unsigned len)
{
    uint32_t crc;
    if (!out || len > PROBE_FRAME_PAYLOAD_MAX || (!payload && len)) return 0;
    out[0] = 0xA5; out[1] = 0x5A; out[2] = 0xC3; out[3] = 0x3C;
    out[4] = type;
    out[5] = (uint8_t)len;
    out[6] = (uint8_t)(len >> 8);
    put_le32(out + 7, session);
    put_le32(out + 11, seq);
    if (len) memcpy(out + 15, payload, len);
    /* CRC covers type .. payload end: 11 fixed bytes + payload. */
    crc = crc_span(0xFFFFFFFFU, out + 4, 11 + len) ^ 0xFFFFFFFFU;
    put_le32(out + 15 + len, crc);
    return 15 + len + 4;
}

void probe_frame_parser_init(probe_frame_parser *p, uint64_t now_us)
{
    memset(p, 0, sizeof(*p));
    p->last_byte_us = now_us;
}

static void reset_to_scan(probe_frame_parser *p)
{
    p->state = STATE_SCAN;
    p->have = 0;
    p->payload_left = 0;
    p->crc_have = 0;
}

static void scan_byte(probe_frame_parser *p, uint8_t byte)
{
    static const uint8_t header[4] = {0xA5, 0x5A, 0xC3, 0x3C};
    if (byte == header[p->have]) {
        p->fixed[p->have++] = byte;
        if (p->have == 4) {
            p->have = 0;
            p->state = STATE_FIXED;
        }
    } else if (byte == header[0]) {
        /* Header has no self-overlap, but a fresh 0xA5 may start the next one. */
        p->fixed[0] = byte;
        p->have = 1;
    } else {
        p->have = 0;
    }
}

/* A rejected candidate header may have swallowed the real frame start: rescan
 * the 18 bytes after its first header byte. Headers buried deeper in a failed
 * frame body are not recovered here; the host request timeout retries. */
static void rescan_prefix(probe_frame_parser *p, int depth)
{
    uint8_t tail[14];
    unsigned j;
    if (depth) { reset_to_scan(p); return; }
    memcpy(tail, p->fixed + 1, sizeof(tail));
    reset_to_scan(p);
    for (j = 0; j < sizeof(tail); ++j)
        scan_byte(p, tail[j]);
}

static int process_byte(probe_frame_parser *p, uint8_t byte, uint64_t now, int depth)
{
    int delivered = 0;
    if (p->state != STATE_SCAN && now - p->last_byte_us > FRAME_GAP_TIMEOUT_US)
        reset_to_scan(p);
    p->last_byte_us = now;
    switch (p->state) {
    case STATE_SCAN:
        scan_byte(p, byte);
        break;
    case STATE_FIXED:
        p->fixed[4 + p->have++] = byte;
        if (p->have == 11) {
            unsigned len = (unsigned)p->fixed[5] | ((unsigned)p->fixed[6] << 8);
            p->type = p->fixed[4];
            if (len > PROBE_FRAME_PAYLOAD_MAX) {
                rescan_prefix(p, depth);
            } else {
                p->payload_len = (uint16_t)len;
                p->payload_left = len;
                p->crc_have = 0;
                p->have = 0;
                p->state = STATE_PAYLOAD;
            }
        }
        break;
    case STATE_PAYLOAD:
        if (p->payload_left) {
            p->payload[p->payload_len - p->payload_left] = byte;
            --p->payload_left;
        } else {
            p->crc_part[p->crc_have++] = byte;
            if (p->crc_have == 4) {
                uint32_t crc = crc_span(crc_span(0xFFFFFFFFU, p->fixed + 4, 11),
                                        p->payload, p->payload_len) ^ 0xFFFFFFFFU;
                if (crc == get_le32(p->crc_part)) {
                    p->session = get_le32(p->fixed + 7);
                    p->seq = get_le32(p->fixed + 11);
                    delivered = 1;
                } else {
                    ++p->crc_errors;
                }
                rescan_prefix(p, depth);
            }
        }
        break;
    default:
        reset_to_scan(p);
        break;
    }
    return delivered;
}

int probe_frame_parse(probe_frame_parser *p, const uint8_t *data, unsigned size, uint64_t now)
{
    int delivered = 0;
    unsigned i;
    for (i = 0; i < size; ++i)
        delivered |= process_byte(p, data[i], now, 0);
    return delivered;
}
