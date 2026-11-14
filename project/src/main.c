/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
  **************************************************************************
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "at32f413_wk_config.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#define LOG_TAG "MAIN"
#include "vibra_log.h"
#include "probe_time.h"
#include "probe_dma.h"
#include "probe_spi.h"
#include "probe_validation.h"
#include "probe_acquisition.h"
#include "probe_usb.h"
#include "probe_capture_check.h"
#include "probe_proto.h"
#include "probe_led.h"
#include "probe_attitude.h"
#include "probe_calib.h"
#include "probe_board_check.h"
#include "probe_stack.h"
/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  probe_stack_init();
  /* add user code begin 1 */

  /* add user code end 1 */

  /* system clock config. */
  wk_system_clock_config();

  /* config periph clock. */
  wk_periph_clock_config();

  /* init debug function. */
  wk_debug_config();

  /* nvic config. */
  wk_nvic_config();

  /* init gpio function. */
  wk_gpio_config();

  /* init spi1 function. */
  wk_spi1_init();

  /* init spi2 function. */
  wk_spi2_init();

  /* init exint function. */
  wk_exint_config();
  /* add user code begin 2 */
	wk_timebase_init();
	probe_time_init();
	probe_dma_init();
	LOG_INIT();
	LOG_I("VibraNode started");
	probe_spi_init();
#if PROBE_HARDWARE_VALIDATION == 1
	probe_validation_init();
#elif PROBE_HARDWARE_VALIDATION == 2
	probe_acquisition_init();
	probe_capture_check_init();
#elif PROBE_HARDWARE_VALIDATION == 3
	probe_board_check_init();
#else
	probe_acquisition_init();
	probe_proto_init();
	probe_led_init();
	probe_attitude_init();
	probe_calib_init();
#endif
  /* add user code end 2 */

  while(1)
  {
    /* add user code begin 3 */
#if PROBE_HARDWARE_VALIDATION == 1
        probe_validation_poll();
#elif PROBE_HARDWARE_VALIDATION == 2
        probe_acquisition_poll();
        probe_capture_check_poll();
#elif PROBE_HARDWARE_VALIDATION == 3
        probe_board_check_poll();
#else
        probe_acquisition_poll();
        probe_attitude_poll();
        probe_proto_poll();
        /* RTT-only link diagnostic (read with J-Link; never on a blocking
         * UART): max_poll_gap answers "did the main loop stall", fifo
         * high-water answers "did reads keep up". */
        {
            static uint64_t next_diag;
            probe_acquisition_status da, di;
            if (probe_time_us() >= next_diag) {
                next_diag = probe_time_us() + 1000000;
                probe_acquisition_get_status(PROBE_SPI_ADXL, &da);
                probe_acquisition_get_status(PROBE_SPI_ICM, &di);
                LOG_I("diag adxl gap=%u q=%u/%u | icm gap=%u fhw=%u q=%u/%u att_drop=%u usb_err=%u",
                      da.max_poll_gap_us, da.queued_batches, da.queued_bytes,
                      di.max_poll_gap_us, di.fifo_highwater, di.queued_batches, di.queued_bytes,
                      di.att_drops, probe_usb_error_count());
            }
        }
#endif
    /* add user code end 3 */
  }
}

  /* add user code begin 4 */

  /* add user code end 4 */
