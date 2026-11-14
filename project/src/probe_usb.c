/**
 * @file probe_usb.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-10-10
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "at32f413_wk_config.h"
#include "probe_time.h"
#include "probe_usb.h"
#include "usbd_core.h"
#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"
#define LOG_TAG "USBLINK"
#include "vibra_log.h"

enum { CHUNK = USBD_CDC_IN_MAXPACKET_SIZE };

typedef struct {
    uint8_t ring[PROBE_USB_RX_RING];
    uint8_t tx[PROBE_USB_TX_MAX];
    uint8_t scratch[CHUNK];
    unsigned read_pos, write_pos, pending;
    unsigned tx_offset, tx_remaining;
    uint64_t tx_deadline_us;
    uint8_t tx_active; /* a frame is staged, not yet fully on the wire */
    uint8_t tx_armed;  /* a packet sits in the endpoint waiting for its TC */
    uint32_t errors;
} probe_usb_context;

static probe_usb_context link;
static usbd_core_type usb_core;

/* Class state owned by cdc_class.c: g_tx_completed is set by the IN
 * completion handler and gates usb_vcp_send_data. */
extern cdc_struct_type cdc_struct;

void usb_delay_us(uint32_t us)
{
    uint64_t deadline = probe_time_us() + us;
    while (probe_time_us() < deadline) { }
}

void usb_delay_ms(uint32_t ms)
{
    usb_delay_us(ms * 1000U);
}

void probe_usb_init(void)
{
    memset(&link, 0, sizeof(link));
    /* USB clock: the internal HICK output, continuously trimmed against the
     * HEXT crystal by the ACC (vendor example flow, usb_clock48m_select's
     * USB_CLK_HICK branch). This frees the PLL from the 48 MHz divisibility
     * constraint, so the core runs at the SPI-optimal 160 MHz with both
     * sensor buses at their spec limits. */
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);
    crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);
    acc_write_c1(7980);
    acc_write_c2(8000);
    acc_write_c3(8020);
    acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);
    crm_periph_clock_enable(CRM_USB_PERIPH_CLOCK, TRUE);
    /* Above the timebase (2), sensor IRQs (3) and DMA (4): the emission
     * burst loop advances one packet per IN completion, so USB latency is
     * wire latency. */
    nvic_irq_enable(USBFS_L_CAN1_RX0_IRQn, 1, 0);
    usbd_core_init(&usb_core, USB, &cdc_class_handler, &cdc_desc_handler, 0);
    usbd_connect(&usb_core);
    LOG_I("USB CDC link ready; HICK 48M USB clock, ACC trimmed");
}

unsigned probe_usb_read(uint8_t *out, unsigned capacity)
{
    unsigned moved = 0;
    if (!out || !capacity) return 0;
    while (moved < capacity && link.pending) {
        out[moved++] = link.ring[link.read_pos];
        link.read_pos = (link.read_pos + 1) % PROBE_USB_RX_RING;
        --link.pending;
    }
    return moved;
}

int probe_usb_send(const uint8_t *frame, unsigned size, uint32_t timeout_us)
{
    unsigned first;
    if (!frame || !size || size > PROBE_USB_TX_MAX || link.tx_active) return 0;
    memcpy(link.tx, frame, size);
    /* usb_vcp_send_data refuses while the previous packet is still on the
     * wire, including the abandoned tail of a timed-out frame. */
    first = size < CHUNK ? size : CHUNK;
    if (usb_vcp_send_data(&usb_core, link.tx, (uint16_t)first) != SUCCESS) return 0;
    link.tx_offset = first;
    link.tx_remaining = size - first;
    link.tx_active = 1;
    link.tx_armed = 1;
    link.tx_deadline_us = probe_time_us() + timeout_us;
    return 1;
}

void probe_usb_poll(void)
{
    uint16_t got;
    /* Each completed OUT packet must be drained before the class re-arms
     * the endpoint, so the loop runs until it reports nothing new. */
    for (;;) {
        got = usb_vcp_get_rxdata(&usb_core, link.scratch);
        if (!got) break;
        if (link.pending + got > PROBE_USB_RX_RING - 1) {
            /* OUT packets NAK until drained, so this only means the protocol
             * layer stopped reading; keep the queued bytes, drop the new. */
            ++link.errors;
            continue;
        }
        {
            unsigned i;
            for (i = 0; i < got; ++i) {
                link.ring[link.write_pos] = link.scratch[i];
                link.write_pos = (link.write_pos + 1) % PROBE_USB_RX_RING;
            }
            link.pending += got;
        }
    }
    /* One 64-byte packet per transmission completion. The frame is done when
     * the last packet's TC has been observed, mirroring the UART "idle only
     * after the wire is quiet" rule the emission burst waits on. */
    while (link.tx_active && link.tx_armed && cdc_struct.g_tx_completed) {
        unsigned chunk;
        link.tx_armed = 0;
        if (!link.tx_remaining) {
            link.tx_active = 0;
            break;
        }
        chunk = link.tx_remaining < CHUNK ? link.tx_remaining : CHUNK;
        if (usb_vcp_send_data(&usb_core, link.tx + link.tx_offset, (uint16_t)chunk) != SUCCESS) {
            /* TC seen yet send refused: cannot happen without concurrent
             * senders; bail out and retry on the next poll. */
            link.tx_armed = 1;
            break;
        }
        link.tx_offset += chunk;
        link.tx_remaining -= chunk;
        link.tx_armed = 1;
    }
    if (link.tx_active && probe_time_us() >= link.tx_deadline_us) {
        /* Host stopped polling: drop the frame. A tail packet may still
         * complete later; send() and this loop handshake through the class
         * completion flag, so the leftover is absorbed safely. */
        link.tx_active = 0;
        link.tx_remaining = 0;
        link.tx_armed = 0;
        ++link.errors;
    }
}

uint32_t probe_usb_error_count(void)
{
    return link.errors;
}

unsigned probe_usb_connected(void)
{
    /* CONFIGURED is reached only when the host sent SET_CONFIGURATION, i.e.
     * its CDC driver bound the interface. Bus suspend (host sleep, cable
     * unplug: SOF loss raises the suspend IRQ) switches to SUSPENDED and a
     * bus reset drops back to DEFAULT until re-enumeration, so both read as
     * "no host" here. */
    return usb_core.conn_state == USB_CONN_STATE_CONFIGURED;
}

int probe_usb_tx_busy(void)
{
    return link.tx_active != 0;
}

void probe_usb_isr(void)
{
    usbd_irq_handler(&usb_core);
}
