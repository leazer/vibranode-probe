/**
 * @file probe_usb.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-10-10
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_USB_H
#define PROBE_USB_H
#include <stdint.h>

/* USB CDC virtual COM link on PA11/PA12 (device, full speed). The protocol
 * layer sees the same shape as the retired RS485 UART: a byte stream in, one
 * frame-buffered transmission out. Full-speed bulk moves 64-byte packets and
 * usbd_ept_send queues exactly one packet per call, so send() stages the frame
 * and poll() feeds the remaining packets as each transmission completes.
 * There is no link rate: the host fixed it at enumeration. */
#define PROBE_USB_RX_RING 1024U
#define PROBE_USB_TX_MAX 1024U

void probe_usb_init(void);
/* Drain newly received bytes; returns how many were copied out. */
unsigned probe_usb_read(uint8_t *out, unsigned capacity);
/* Arms one transmission; returns 0 when busy or arguments invalid. */
int probe_usb_send(const uint8_t *frame, unsigned size, uint32_t timeout_us);
/* Main-loop service: RX packets into the ring, next TX packet on completion. */
void probe_usb_poll(void);
uint32_t probe_usb_error_count(void);
int probe_usb_tx_busy(void);
/* Nonzero once the host finished enumeration (SET_CONFIGURATION). Suspend
 * (host sleep, unplug) and bus reset read as zero until re-enumeration. */
unsigned probe_usb_connected(void);
/* USBFS_L_CAN1_RX0_IRQHandler body. */
void probe_usb_isr(void);

#endif
