/**
 * @file probe_frame.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_FRAME_H
#define PROBE_FRAME_H
#include <stdint.h>

/* Frozen RS-485 frame codec: docs/superpowers/specs/2026-09-07-vibranode-rs485-protocol-design.md.
 * Chip-independent: no AT32, RTT or transport dependencies. */
#define PROBE_FRAME_MAX 1024U
#define PROBE_FRAME_PAYLOAD_MAX 1005U

/* Request types the probe receives; responses reuse the same values. */
enum {
    PROBE_FRAME_HELLO = 0x01,
    PROBE_FRAME_SET_LINK = 0x02,
    PROBE_FRAME_SET_CONFIG = 0x03,
    PROBE_FRAME_START = 0x04,
    PROBE_FRAME_FETCH = 0x05,
    PROBE_FRAME_STOP = 0x06,
    PROBE_FRAME_CALIBRATE = 0x07,
    PROBE_FRAME_SAVE_CAL = 0x08,
    PROBE_FRAME_GET_STATUS = 0x09
};

uint32_t probe_frame_crc32c(const uint8_t *data, unsigned size);
/* Returns the total frame size, or 0 for invalid arguments. */
unsigned probe_frame_build(uint8_t *out, uint8_t type, uint32_t session, uint32_t seq,
                           const uint8_t *payload, unsigned len);

typedef struct {
    /* Valid when probe_frame_parse returns 1 (fields of the last frame only). */
    uint8_t type;
    uint32_t session, seq;
    uint16_t payload_len;
    uint8_t payload[PROBE_FRAME_PAYLOAD_MAX];
    /* Link diagnostics for the protocol layer. Type and per-type payload
     * validation is role knowledge and lives in the protocol layer. */
    uint32_t crc_errors;
    /* Internal parser state; treat as private. */
    unsigned state, have, payload_left, crc_have;
    uint8_t fixed[15]; /* Header 4 + type/len/session/seq 11 */
    uint8_t crc_part[4];
    uint64_t last_byte_us;
} probe_frame_parser;

void probe_frame_parser_init(probe_frame_parser *p, uint64_t now_us);
/* Feed received bytes; returns 1 if at least one complete valid frame was
 * delivered into the fields above (later frames overwrite earlier ones). */
int probe_frame_parse(probe_frame_parser *p, const uint8_t *data, unsigned size, uint64_t now_us);

#endif
