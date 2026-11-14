/**
 * @file probe_proto.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_PROTO_H
#define PROBE_PROTO_H

/* Protocol state machine (frozen on RS-485, now carried over the USB CDC
 * link): docs/superpowers/specs/2026-09-07-vibranode-rs485-protocol-design.md.
 * Owns session, request idempotency, measurement config and FETCH windows.
 * Transport (probe_usb) and acquisition are consumed through their APIs. */
void probe_proto_init(void);
void probe_proto_poll(void);

#endif
