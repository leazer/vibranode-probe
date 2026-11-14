/**
 * @file test_probe_frame.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "probe_frame.h"

#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

/* Frozen shared vectors from the protocol spec, section 10. */
static const uint8_t frame_a[27] = {
    0xA5, 0x5A, 0xC3, 0x3C, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x44, 0x33, 0x22,
    0x11, 0xBC, 0xF0, 0x6B, 0x2F};
static const uint8_t frame_b[60] = {
    0xA5, 0x5A, 0xC3, 0x3C, 0x10, 0x29, 0x00, 0x44, 0x33, 0x22, 0x11,
    0x07, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x42,
    0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0x00, 0x01,
    0x00, 0x03, 0x00, 0xD8, 0xF5, 0x0F, 0x8C, 0x0A, 0x00, 0x24, 0xC2,
    0x00, 0x18, 0x19, 0xC2, 0xBA};

static unsigned build_stream_b_payload(uint8_t *out)
{
    unsigned i = 0, axis;
    out[i++] = 1; out[i++] = 0; out[i++] = 0; out[i++] = 0;   /* config id */
    out[i++] = 1; out[i++] = 0; out[i++] = 0; out[i++] = 0;   /* segment */
    memset(out + i, 0, 8); i += 8;                            /* first sequence */
    out[i++] = 0x40; out[i++] = 0x42; out[i++] = 0x0F;        /* 1000000 us */
    out[i++] = 0; out[i++] = 0; out[i++] = 0; out[i++] = 0; out[i++] = 0;
    out[i++] = 0x00; out[i++] = 0x00; out[i++] = 0xFA; out[i++] = 0x00; /* 250 us Q16.16 */
    out[i++] = 1; out[i++] = 0;                               /* count */
    out[i++] = 0x03; out[i++] = 0x00;                         /* quality flags */
    /* 20-bit two's complement little-endian: -2600, 2700, 49700 */
    {
        static const int32_t values[3] = {-2600, 2700, 49700};
        for (axis = 0; axis < 3; ++axis) {
            uint32_t raw = (uint32_t)values[axis] & 0xFFFFFU;
            out[i++] = (uint8_t)raw;
            out[i++] = (uint8_t)(raw >> 8);
            out[i++] = (uint8_t)(raw >> 16);
        }
    }
    return i;
}

static unsigned feed_and_collect(probe_frame_parser *p, const uint8_t *data,
                                 unsigned size, uint64_t now, uint8_t *out_type,
                                 uint32_t *out_session)
{
    if (!probe_frame_parse(p, data, size, now)) return 0;
    *out_type = p->type;
    *out_session = p->session;
    return p->payload_len;
}

int test_main(void)
{
    probe_frame_parser parser;
    uint8_t buffer[PROBE_FRAME_MAX];
    uint8_t payload[PROBE_FRAME_PAYLOAD_MAX];
    uint8_t decoded_type;
    uint32_t decoded_session;
    unsigned len, crc_errors, i;

    /* 1. CRC self-check vector. */
    CHECK(probe_frame_crc32c((const uint8_t *)"123456789", 9) == 0xE3069283U);

    /* 2. Build frame A byte-for-byte. */
    payload[0] = 0x01; payload[1] = 0; payload[2] = 0; payload[3] = 0;
    payload[4] = 0x44; payload[5] = 0x33; payload[6] = 0x22; payload[7] = 0x11;
    len = probe_frame_build(buffer, 0x01, 0, 0, payload, 8);
    CHECK(len == sizeof(frame_a));
    CHECK(memcmp(buffer, frame_a, sizeof(frame_a)) == 0);

    /* 3. Parse frame A. */
    probe_frame_parser_init(&parser, 1000);
    len = feed_and_collect(&parser, frame_a, sizeof(frame_a), 1001,
                           &decoded_type, &decoded_session);
    CHECK(len == 8 && decoded_type == 0x01 && decoded_session == 0);
    CHECK(parser.seq == 0);
    CHECK(parser.payload[4] == 0x44 && parser.payload[7] == 0x11);

    /* 4. Build and parse frame B (stream payload with negative 20-bit sample). */
    len = build_stream_b_payload(payload);
    CHECK(len == 41);
    len = probe_frame_build(buffer, 0x10, 0x11223344U, 7, payload, len);
    CHECK(len == sizeof(frame_b));
    CHECK(memcmp(buffer, frame_b, sizeof(frame_b)) == 0);
    probe_frame_parser_init(&parser, 0);
    len = feed_and_collect(&parser, frame_b, sizeof(frame_b), 1,
                           &decoded_type, &decoded_session);
    CHECK(len == 41 && decoded_type == 0x10 && decoded_session == 0x11223344U);
    CHECK(parser.seq == 7);
    /* Sign extension check on the first sample axis. */
    CHECK(parser.payload[32] == 0xD8 && parser.payload[33] == 0xF5 &&
          parser.payload[34] == 0x0F);

    /* 5. Garbage and a partial header in front of a good frame. */
    probe_frame_parser_init(&parser, 0);
    {
        uint8_t noisy[64];
        memset(noisy, 0x00, sizeof(noisy));
        noisy[10] = 0xA5; noisy[11] = 0x5A; noisy[12] = 0xC3; /* incomplete header */
        CHECK(probe_frame_parse(&parser, noisy, sizeof(noisy), 1) == 0);
        len = feed_and_collect(&parser, frame_a, sizeof(frame_a), 2,
                               &decoded_type, &decoded_session);
        CHECK(len == 8 && decoded_type == 0x01);
    }

    /* 6. Bad CRC frame is dropped and counted; the next frame still arrives. */
    probe_frame_parser_init(&parser, 0);
    crc_errors = parser.crc_errors;
    memcpy(buffer, frame_a, sizeof(frame_a));
    buffer[26] ^= 0xFF;
    CHECK(probe_frame_parse(&parser, buffer, sizeof(frame_a), 1) == 0);
    CHECK(parser.crc_errors == crc_errors + 1);
    len = feed_and_collect(&parser, frame_a, sizeof(frame_a), 2,
                           &decoded_type, &decoded_session);
    CHECK(len == 8);

    /* 7. Length field beyond the payload cap is skipped without derailment. */
    probe_frame_parser_init(&parser, 0);
    memcpy(buffer, frame_a, sizeof(frame_a));
    buffer[5] = 0xFF; buffer[6] = 0x0F; /* claims 4095 bytes */
    CHECK(probe_frame_parse(&parser, buffer, sizeof(frame_a), 1) == 0);
    len = feed_and_collect(&parser, frame_a, sizeof(frame_a), 2,
                           &decoded_type, &decoded_session);
    CHECK(len == 8);

    /* 8. Payload beyond a request's cap is still structurally valid; the
     * protocol layer rejects it. The parser must deliver it untouched. */
    probe_frame_parser_init(&parser, 0);
    payload[0] = 0x01; memset(payload + 1, 0, 13);
    len = probe_frame_build(buffer, 0x03, 0x11223344U, 0, payload, 14);
    CHECK(len == 33);
    len = feed_and_collect(&parser, buffer, len, 1, &decoded_type, &decoded_session);
    CHECK(len == 14 && decoded_type == 0x03);

    /* 9. Unknown types deliver too; role-specific counting is protocol work. */
    probe_frame_parser_init(&parser, 0);
    len = probe_frame_build(buffer, 0x7F, 0x11223344U, 0, payload, 4);
    CHECK(len == 23);
    len = feed_and_collect(&parser, buffer, len, 1, &decoded_type, &decoded_session);
    CHECK(len == 4 && decoded_type == 0x7F);

    /* 10. A gap beyond 50 ms mid-frame resets the parser; later frames survive. */
    probe_frame_parser_init(&parser, 0);
    CHECK(probe_frame_parse(&parser, frame_a, 10, 1000) == 0);
    CHECK(probe_frame_parse(&parser, frame_a + 10, sizeof(frame_a) - 10, 52000) == 0);
    len = feed_and_collect(&parser, frame_a, sizeof(frame_a), 52001,
                           &decoded_type, &decoded_session);
    CHECK(len == 8);

    /* 11. Build rejects oversized payloads. */
    CHECK(probe_frame_build(buffer, 0x10, 1, 0, payload, 1006) == 0);

    /* 12. Fragmented delivery byte-by-byte across calls. */
    probe_frame_parser_init(&parser, 0);
    {
        int delivered = 0;
        for (i = 0; i < sizeof(frame_b); ++i)
            delivered |= probe_frame_parse(&parser, frame_b + i, 1, i);
        CHECK(delivered);
    }
    CHECK(parser.payload_len == 41 && parser.type == 0x10);
    return 0;
}
