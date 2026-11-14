/**
 * @file usb_conf.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-10-10
 *
 * @copyright Copyright (c) 2026
 *
 */
/**
  **************************************************************************
  * @file     usb_conf.h
  * @brief    usb endpoint configuration for the VibraNode CDC link
  **************************************************************************
  * Adapted from the AT32F413 firmware library V2.1.7 usb_device
  * vcp_loopback example: same endpoint buffer map, without the board BSP
  * dependency. usb_delay_ms/us are implemented in probe_usb.c on top of
  * probe_time.
  **************************************************************************
  */

#ifndef __USB_CONF_H
#define __USB_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f413.h"
#include "stdio.h"

/* usb device support endpoint number */
#ifndef USB_EPT_MAX_NUM
#define USB_EPT_MAX_NUM                   8
#endif

/* usb enable extend buffer (768-1280 bytes); CDC fits the default 512 B */
/* #define USB_BUFFER_SIZE_EX */

/* usb auto malloc endpoint tx and rx buffer */
/* #define USB_EPT_AUTO_MALLOC_BUFFER */

#ifndef USB_EPT_AUTO_MALLOC_BUFFER
/**
 * user custom endpoint buffer
 *        EPTn_TX_ADDR, EPTn_RX_ADDR must less than usb buffer size
 */

/* ept0 tx start address 0x40, size 0x40,
   so rx start address is 0x40 + 0x40 = 0x80 */
#define EPT0_TX_ADDR                     0x40    /*!< usb endpoint 0 tx buffer address offset */
#define EPT0_RX_ADDR                     0x80    /*!< usb endpoint 0 rx buffer address offset */

#define EPT1_TX_ADDR                     0xC0    /*!< usb endpoint 1 tx buffer address offset */
#define EPT1_RX_ADDR                     0x100   /*!< usb endpoint 1 rx buffer address offset */

#define EPT2_TX_ADDR                     0x140   /*!< usb endpoint 2 tx buffer address offset */
#define EPT2_RX_ADDR                     0x180   /*!< usb endpoint 2 rx buffer address offset */

#define EPT3_TX_ADDR                     0x00    /*!< usb endpoint 3 tx buffer address offset */
#define EPT3_RX_ADDR                     0x00    /*!< usb endpoint 3 rx buffer address offset */

#define EPT4_TX_ADDR                     0x00    /*!< usb endpoint 4 tx buffer address offset */
#define EPT4_RX_ADDR                     0x00    /*!< usb endpoint 4 rx buffer address offset */

#define EPT5_TX_ADDR                     0x00    /*!< usb endpoint 5 tx buffer address offset */
#define EPT5_RX_ADDR                     0x00    /*!< usb endpoint 5 rx buffer address offset */

#define EPT6_TX_ADDR                     0x00    /*!< usb endpoint 6 tx buffer address offset */
#define EPT6_RX_ADDR                     0x00    /*!< usb endpoint 6 rx buffer address offset */

#define EPT7_TX_ADDR                     0x00    /*!< usb endpoint 7 tx buffer address offset */
#define EPT7_RX_ADDR                     0x00    /*!< usb endpoint 7 rx buffer address offset */

#endif

void usb_delay_ms(uint32_t ms);
void usb_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif
