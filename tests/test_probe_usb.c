/**
 * @file test_probe_usb.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-10-10
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "at32f413.h"
#include "probe_usb.h"
#include "cdc_class.h"
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

void test_log_info(const char *fmt, ...) { (void)fmt; }
void test_log_error(const char *fmt, ...) { (void)fmt; }

/* ---- hardware / middleware fakes ---- */
static uint64_t now_us;
static unsigned core_init_count, connect_count, irq_count, clock_selected;
static IRQn_Type nvic_irq;

uint64_t probe_time_us(void) { return now_us; }
void crm_periph_clock_enable(uint32_t periph, confirm_state state) { (void)periph; (void)state; }
void crm_usb_clock_source_select(crm_usb_clock_source_type source) { (void)source; ++clock_selected; }
void acc_write_c1(uint16_t value) { (void)value; }
void acc_write_c2(uint16_t value) { (void)value; }
void acc_write_c3(uint16_t value) { (void)value; }
void acc_calibration_mode_enable(uint16_t mode, confirm_state state)
{
    (void)mode;
    if (state) ++clock_selected;
}
void nvic_irq_enable(IRQn_Type irq, uint32_t preempt, uint32_t sub)
{
    (void)preempt; (void)sub;
    nvic_irq = irq;
}

/* The real core/class are not linked in; emulate the call topology the glue
 * depends on. cdc_struct is the vendor global normally defined in cdc_class.c
 * and is owned by this test. */
cdc_struct_type cdc_struct;
usbd_class_handler cdc_class_handler;
usbd_desc_handler cdc_desc_handler;
void usbd_core_init(usbd_core_type *udev, usb_reg_type *reg,
                    usbd_class_handler *cls, usbd_desc_handler *desc, uint8_t id)
{
    (void)udev; (void)reg; (void)cls; (void)desc; (void)id;
    ++core_init_count;
}
void usbd_connect(usbd_core_type *udev) { (void)udev; ++connect_count; }
void usbd_irq_handler(usbd_core_type *udev) { (void)udev; ++irq_count; }

/* usb_vcp_send_data: refuses while the previous packet lacks its completion,
 * writes one <=64 B packet, clears the completion flag. */
static struct {
    uint8_t log[32][64];
    uint16_t log_len[32];
    unsigned count;
    unsigned refuse;
} tx;
error_status usb_vcp_send_data(void *udev, uint8_t *data, uint16_t len)
{
    (void)udev;
    if (!cdc_struct.g_tx_completed || tx.refuse || len > 64) return ERROR;
    memcpy(tx.log[tx.count], data, len);
    tx.log_len[tx.count] = len;
    ++tx.count;
    cdc_struct.g_tx_completed = 0;
    return SUCCESS;
}

/* usb_vcp_get_rxdata: serves test-programmed packets, one per call. */
static struct {
    uint8_t packets[20][64];
    uint16_t len[20];
    unsigned head, count;
} rx;
uint16_t usb_vcp_get_rxdata(void *udev, uint8_t *out)
{
    (void)udev;
    if (!rx.count) return 0;
    memcpy(out, rx.packets[rx.head], rx.len[rx.head]);
    --rx.count;
    rx.head = (rx.head + 1) % 20;
    return rx.len[(rx.head + 19) % 20];
}

static void offer_rx(const void *data, unsigned len)
{
    unsigned i;
    for (i = 0; i < len; ++i)
        rx.packets[(rx.head + rx.count) % 20][i] = ((const uint8_t *)data)[i];
    rx.len[(rx.head + rx.count) % 20] = (uint16_t)len;
    ++rx.count;
}

/* Host finished one IN packet: the class completion handler runs in the USB
 * interrupt in the real system. */
static void host_polled(void) { cdc_struct.g_tx_completed = 1; }

int test_main(void)
{
    uint8_t frame[1024], out[1024];
    unsigned i;
    uint64_t errors;

    /* a. init: HICK 48M clock select + ACC trim, USB vector priority slot,
     * core init + connect. */
    probe_usb_init();
    CHECK(core_init_count == 1 && connect_count == 1);
    CHECK(clock_selected == 2); /* source select + ACC calibration on */
    CHECK(nvic_irq == USBFS_L_CAN1_RX0_IRQn);
    CHECK(!probe_usb_tx_busy());
    probe_usb_isr();
    CHECK(irq_count == 1);
    cdc_struct.g_tx_completed = 1; /* class init default */

    /* b. a 1024 B frame leaves as 16 packets of 64 B, in order, and the link
     * stays busy until the final packet's completion is observed. */
    for (i = 0; i < sizeof(frame); ++i) frame[i] = (uint8_t)(i * 7U);
    CHECK(probe_usb_send(frame, sizeof(frame), 200000));
    CHECK(probe_usb_tx_busy());
    CHECK(tx.count == 1 && tx.log_len[0] == 64);
    for (i = 0; i < 64; ++i) CHECK(tx.log[0][i] == frame[i]);
    CHECK(probe_usb_send(frame, sizeof(frame), 200000) == 0); /* busy */
    now_us += 10;
    probe_usb_poll(); /* completion not yet observed: no new packet */
    CHECK(tx.count == 1);
    host_polled();
    probe_usb_poll();
    CHECK(tx.count == 2);
    CHECK(probe_usb_tx_busy());
    while (probe_usb_tx_busy()) {
        host_polled();
        probe_usb_poll();
    }
    CHECK(tx.count == 16);
    for (i = 1; i < 16; ++i) {
        unsigned j;
        CHECK(tx.log_len[i] == 64);
        for (j = 0; j < 64; ++j) CHECK(tx.log[i][j] == frame[i * 64U + j]);
    }
    errors = probe_usb_error_count();
    CHECK(errors == 0);

    /* c. short tail: 100 B -> 64 + 36. */
    tx.count = 0;
    CHECK(probe_usb_send(frame, 100, 200000));
    host_polled(); probe_usb_poll();
    while (probe_usb_tx_busy()) { host_polled(); probe_usb_poll(); }
    CHECK(tx.count == 2 && tx.log_len[0] == 64 && tx.log_len[1] == 36);

    /* d. rejected sends: zero, oversized, and refused first packet. */
    CHECK(probe_usb_send(frame, 0, 200000) == 0);
    CHECK(probe_usb_send(frame, PROBE_USB_TX_MAX + 1, 200000) == 0);
    tx.refuse = 1;
    CHECK(probe_usb_send(frame, 64, 200000) == 0);
    tx.refuse = 0;

    /* e. RX: packets land in the ring in order and drain via read. */
    for (i = 0; i < 158; ++i) frame[i] = (uint8_t)(i + 3U);
    offer_rx(frame, 64);
    offer_rx(frame + 64, 64);
    offer_rx(frame + 128, 30);
    probe_usb_poll();
    CHECK(probe_usb_read(out, 64) == 64);
    CHECK(probe_usb_read(out + 64, 1000) == 94);
    for (i = 0; i < 158; ++i) CHECK(out[i] == (uint8_t)(i + 3U));
    CHECK(probe_usb_read(out, 64) == 0);
    CHECK(probe_usb_error_count() == 0);

    /* f. ring overflow drops the packets that no longer fit, one error each. */
    for (i = 0; i < 17; ++i) offer_rx(frame, 64);
    probe_usb_poll();
    CHECK(probe_usb_read(out, sizeof(out)) == 960);
    CHECK(probe_usb_error_count() == 2);

    /* g. deadline abort: the frame is dropped and an error counted; a late
     * completion from the abandoned tail still lets a fresh frame start. */
    CHECK(probe_usb_send(frame, 1024, 1000));
    now_us += 1001;
    probe_usb_poll();
    CHECK(!probe_usb_tx_busy());
    CHECK(probe_usb_error_count() == 3);
    host_polled(); /* abandoned tail completes only now */
    CHECK(probe_usb_send(frame, 128, 1000));
    while (probe_usb_tx_busy()) { host_polled(); probe_usb_poll(); }
    CHECK(tx.count >= 2);

    return 0;
}
