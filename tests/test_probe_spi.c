/**
 * @file test_probe_spi.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "at32f413.h"
#include "probe_spi.h"

typedef struct {
    uint64_t now;
    uint32_t dma_flags;
    uint32_t spi_flags[2];
    uint32_t cs_high[2];
    uint32_t enabled[4];
    uint32_t reset_count[2];
    dma_init_type transfers[4];
} test_hardware;
static test_hardware hw;
static unsigned bus_index(spi_type *p) { return p == SPI1 ? 0 : 1; }
static unsigned channel_index(dma_channel_type *p)
{
    if (p == DMA1_CHANNEL1) return 0;
    if (p == DMA1_CHANNEL2) return 1;
    if (p == DMA1_CHANNEL3) return 2;
    return 3;
}
uint64_t probe_time_us(void) { return hw.now; }
void gpio_bits_set(gpio_type *p, uint16_t bits) { (void)bits; hw.cs_high[p == GPIOA ? 0 : 1] = 1; }
void gpio_bits_reset(gpio_type *p, uint16_t bits) { (void)bits; hw.cs_high[p == GPIOA ? 0 : 1] = 0; }
void spi_i2s_reset(spi_type *p) { unsigned i = bus_index(p); hw.spi_flags[i] = 0; ++hw.reset_count[i]; }
void spi_default_para_init(spi_init_type *p) { memset(p, 0, sizeof(*p)); }
void spi_init(spi_type *p, spi_init_type *v) { (void)p; (void)v; }
void spi_enable(spi_type *p, confirm_state value) { (void)p; (void)value; }
void spi_i2s_dma_receiver_enable(spi_type *p, confirm_state value) { (void)p; (void)value; }
void spi_i2s_dma_transmitter_enable(spi_type *p, confirm_state value) { (void)p; (void)value; }
flag_status spi_i2s_flag_get(spi_type *p, uint32_t flag) { return (hw.spi_flags[bus_index(p)] & flag) ? SET : RESET; }
void dma_default_para_init(dma_init_type *p) { memset(p, 0, sizeof(*p)); }
void dma_init(dma_channel_type *p, dma_init_type *v) { hw.transfers[channel_index(p)] = *v; }
void dma_channel_enable(dma_channel_type *p, confirm_state value) { hw.enabled[channel_index(p)] = value; }
void dma_interrupt_enable(dma_channel_type *p, uint32_t flags, confirm_state value) { (void)p; (void)flags; (void)value; }
flag_status dma_flag_get(uint32_t flag) { return (hw.dma_flags & flag) ? SET : RESET; }
void dma_flag_clear(uint32_t flags) { hw.dma_flags &= ~flags; }
void nvic_irq_enable(IRQn_Type irq, uint32_t preempt, uint32_t sub) { (void)irq; (void)preempt; (void)sub; }

#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
int test_main(void)
{
    uint8_t command[3] = {0x81, 0, 0};
    uint8_t other[2] = {0xF5, 0};
    probe_spi_init();
    CHECK(probe_spi_start(PROBE_SPI_COUNT, command, 3, 2000) == PROBE_SPI_INVALID);
    CHECK(probe_spi_start(PROBE_SPI_ADXL, 0, 3, 2000) == PROBE_SPI_INVALID);
    CHECK(probe_spi_start(PROBE_SPI_ADXL, command, 0, 2000) == PROBE_SPI_INVALID);
    CHECK(probe_spi_start(PROBE_SPI_ADXL, command, 1026, 2000) == PROBE_SPI_INVALID);
    CHECK(probe_spi_start(PROBE_SPI_ADXL, command, 3, 0) == PROBE_SPI_INVALID);
    CHECK(probe_spi_start(PROBE_SPI_ADXL, command, 3, 2000) == PROBE_SPI_PENDING);
    command[0] = 0;
    CHECK(*(const uint8_t *)hw.transfers[1].memory_base_addr == 0x81);
    CHECK(hw.transfers[0].direction == DMA_DIR_PERIPHERAL_TO_MEMORY);
    CHECK(hw.transfers[1].direction == DMA_DIR_MEMORY_TO_PERIPHERAL);
    CHECK(hw.transfers[0].buffer_size == 3 && !hw.cs_high[0]);
    CHECK(probe_spi_start(PROBE_SPI_ADXL, command, 3, 2000) == PROBE_SPI_BUSY);
    CHECK(*(const uint8_t *)hw.transfers[1].memory_base_addr == 0x81);
    CHECK(probe_spi_start(PROBE_SPI_ICM, other, 2, 2000) == PROBE_SPI_PENDING);
    hw.dma_flags = DMA1_FDT1_FLAG;
    probe_spi_dma_isr(PROBE_SPI_ADXL);
    CHECK(probe_spi_poll(PROBE_SPI_ADXL) == PROBE_SPI_PENDING && !hw.cs_high[0]);
    hw.dma_flags = DMA1_FDT2_FLAG;
    hw.spi_flags[0] = SPI_I2S_BF_FLAG;
    probe_spi_dma_isr(PROBE_SPI_ADXL);
    CHECK(probe_spi_poll(PROBE_SPI_ADXL) == PROBE_SPI_PENDING && !hw.cs_high[0]);
    hw.spi_flags[0] = 0;
    CHECK(probe_spi_poll(PROBE_SPI_ADXL) == PROBE_SPI_DONE && hw.cs_high[0]);
    CHECK(!hw.enabled[0] && !hw.enabled[1]);
    CHECK(!hw.cs_high[1]); /* no cross-bus cancellation */
    hw.dma_flags = DMA1_DTERR4_FLAG;
    probe_spi_dma_isr(PROBE_SPI_ICM);
    CHECK(probe_spi_poll(PROBE_SPI_ICM) == PROBE_SPI_ERROR && hw.cs_high[1]);
    CHECK(probe_spi_rx(PROBE_SPI_ICM) == 0);
    CHECK(probe_spi_start(PROBE_SPI_ICM, other, 2, 2000) == PROBE_SPI_PENDING);
    hw.now = 2000;
    CHECK(probe_spi_poll(PROBE_SPI_ICM) == PROBE_SPI_TIMEOUT && hw.cs_high[1]);
    CHECK(hw.reset_count[1] >= 2);
    CHECK(probe_spi_start(PROBE_SPI_ICM, other, 2, 2000) == PROBE_SPI_PENDING);
    hw.spi_flags[1] = SPI_I2S_ROERR_FLAG;
    CHECK(probe_spi_poll(PROBE_SPI_ICM) == PROBE_SPI_ERROR);
    CHECK(probe_spi_read(PROBE_SPI_ICM, 0xB0, 0, 2000) == PROBE_SPI_INVALID);
    CHECK(probe_spi_read(PROBE_SPI_ICM, 0xB0, 1025, 2000) == PROBE_SPI_INVALID);
    CHECK(probe_spi_read(PROBE_SPI_ICM, 0xB0, 512, 2000) == PROBE_SPI_PENDING);
    CHECK(hw.transfers[3].buffer_size == 513); /* 1 command + 512 payload */
    CHECK(((const uint8_t *)hw.transfers[3].memory_base_addr)[0] == 0xB0);
    CHECK(((const uint8_t *)hw.transfers[3].memory_base_addr)[512] == 0);
    return 0;
}
