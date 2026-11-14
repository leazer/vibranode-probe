/**
 * @file probe_spi.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "at32f413_wk_config.h"
#include "probe_spi.h"
#include "probe_time.h"

enum { EVENT_RX_DONE = 1, EVENT_TX_DONE = 2, EVENT_ERROR = 4 };

typedef struct {
    spi_type *spi;
    gpio_type *cs_port;
    uint16_t cs_pin;
    dma_channel_type *rx_dma;
    dma_channel_type *tx_dma;
    uint32_t rx_done, tx_done, errors, clear_flags;
    IRQn_Type rx_irq, tx_irq;
    spi_mclk_freq_div_type divider;
} probe_spi_hardware;

typedef struct {
    uint8_t tx[PROBE_SPI_MAX_BYTES];
    uint8_t rx[PROBE_SPI_MAX_BYTES];
    uint64_t deadline_us;
    volatile uint32_t events;
    probe_spi_result result;
} probe_spi_context;

/* APB1/APB2 run at 100 MHz (part maximum). With the HICK USB clock the core
 * runs at the 200 MHz part maximum; SPI2 20 MHz (100/4) and SPI1 10 MHz
 * (100/8) would also be inside the sensor spec limits, but board round 19
 * showed the core needs the headroom more than the SPI speed, so the buses
 * stay at 12.5 MHz (100/8) and 6.25 MHz (100/16). */
static const probe_spi_hardware hardware[PROBE_SPI_COUNT] = {
    {SPI1, ADXL_CS_GPIO_PORT, ADXL_CS_PIN, DMA1_CHANNEL1, DMA1_CHANNEL2,
     DMA1_FDT1_FLAG, DMA1_FDT2_FLAG, DMA1_DTERR1_FLAG | DMA1_DTERR2_FLAG,
     0xFFU, DMA1_Channel1_IRQn, DMA1_Channel2_IRQn, SPI_MCLK_DIV_16},
    {SPI2, IMU_CS_GPIO_PORT, IMU_CS_PIN, DMA1_CHANNEL3, DMA1_CHANNEL4,
     DMA1_FDT3_FLAG, DMA1_FDT4_FLAG, DMA1_DTERR3_FLAG | DMA1_DTERR4_FLAG,
     0xFF00U, DMA1_Channel3_IRQn, DMA1_Channel4_IRQn, SPI_MCLK_DIV_8}
};
static probe_spi_context buses[PROBE_SPI_COUNT];

static void configure_spi(const probe_spi_hardware *hw)
{
    spi_init_type config;
    spi_i2s_reset(hw->spi);
    spi_default_para_init(&config);
    config.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    config.master_slave_mode = SPI_MODE_MASTER;
    config.frame_bit_num = SPI_FRAME_8BIT;
    config.first_bit_transmission = SPI_FIRST_BIT_MSB;
    config.mclk_freq_division = hw->divider;
    config.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    config.clock_phase = SPI_CLOCK_PHASE_1EDGE;
    config.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(hw->spi, &config);
    spi_enable(hw->spi, TRUE);
}

static void stop_transfer(const probe_spi_hardware *hw)
{
    spi_i2s_dma_transmitter_enable(hw->spi, FALSE);
    spi_i2s_dma_receiver_enable(hw->spi, FALSE);
    dma_channel_enable(hw->tx_dma, FALSE);
    dma_channel_enable(hw->rx_dma, FALSE);
    dma_interrupt_enable(hw->rx_dma, DMA_FDT_INT | DMA_DTERR_INT, FALSE);
    dma_interrupt_enable(hw->tx_dma, DMA_FDT_INT | DMA_DTERR_INT, FALSE);
    dma_flag_clear(hw->clear_flags);
}

void probe_spi_init(void)
{
    unsigned i;
    for (i = 0; i < PROBE_SPI_COUNT; ++i) {
        stop_transfer(&hardware[i]);
        gpio_bits_set(hardware[i].cs_port, hardware[i].cs_pin);
        configure_spi(&hardware[i]);
        memset(&buses[i], 0, sizeof(buses[i]));
        nvic_irq_enable(hardware[i].rx_irq, 4, 0);
        nvic_irq_enable(hardware[i].tx_irq, 4, 0);
    }
}

static probe_spi_result start_transfer(probe_spi_bus bus, const uint8_t *tx,
                                       uint16_t size, uint32_t timeout_us,
                                       int read, uint8_t command)
{
    const probe_spi_hardware *hw;
    probe_spi_context *ctx;
    dma_init_type config;
    if ((unsigned)bus >= PROBE_SPI_COUNT || (!read && !tx) || !size ||
        size > PROBE_SPI_MAX_BYTES || !timeout_us)
        return PROBE_SPI_INVALID;
    ctx = &buses[bus];
    hw = &hardware[bus];
    if (ctx->result == PROBE_SPI_PENDING) return PROBE_SPI_BUSY;

    stop_transfer(hw);
    ctx->events = 0;
    if (read) {
        memset(ctx->tx, 0, size);
        ctx->tx[0] = command;
    } else {
        memcpy(ctx->tx, tx, size);
    }
    ctx->deadline_us = probe_time_us() + timeout_us;
    dma_default_para_init(&config);
    config.peripheral_base_addr = (uint32_t)&hw->spi->dt;
    config.memory_base_addr = (uint32_t)ctx->rx;
    config.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    config.buffer_size = size;
    config.memory_inc_enable = TRUE;
    config.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    config.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    config.priority = DMA_PRIORITY_VERY_HIGH;
    dma_init(hw->rx_dma, &config);
    config.memory_base_addr = (uint32_t)ctx->tx;
    config.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    config.priority = DMA_PRIORITY_HIGH;
    dma_init(hw->tx_dma, &config);
    dma_interrupt_enable(hw->rx_dma, DMA_FDT_INT | DMA_DTERR_INT, TRUE);
    dma_interrupt_enable(hw->tx_dma, DMA_FDT_INT | DMA_DTERR_INT, TRUE);
    ctx->result = PROBE_SPI_PENDING;
    gpio_bits_reset(hw->cs_port, hw->cs_pin);
    /* RX is armed before TX can produce clocks. */
    dma_channel_enable(hw->rx_dma, TRUE);
    spi_i2s_dma_receiver_enable(hw->spi, TRUE);
    dma_channel_enable(hw->tx_dma, TRUE);
    spi_i2s_dma_transmitter_enable(hw->spi, TRUE);
    return PROBE_SPI_PENDING;
}

probe_spi_result probe_spi_start(probe_spi_bus bus, const uint8_t *tx,
                                 uint16_t size, uint32_t timeout_us)
{
    return start_transfer(bus, tx, size, timeout_us, 0, 0);
}

probe_spi_result probe_spi_read(probe_spi_bus bus, uint8_t command,
                                uint16_t payload_size, uint32_t timeout_us)
{
    if (!payload_size || payload_size >= PROBE_SPI_MAX_BYTES) return PROBE_SPI_INVALID;
    return start_transfer(bus, 0, payload_size + 1, timeout_us, 1, command);
}

void probe_spi_dma_isr(probe_spi_bus bus)
{
    const probe_spi_hardware *hw;
    uint32_t events = 0, clear = 0;
    if ((unsigned)bus >= PROBE_SPI_COUNT) return;
    hw = &hardware[bus];
    if (dma_flag_get(hw->rx_done) == SET) { events |= EVENT_RX_DONE; clear |= hw->rx_done; }
    if (dma_flag_get(hw->tx_done) == SET) { events |= EVENT_TX_DONE; clear |= hw->tx_done; }
    if (dma_flag_get(hw->errors) == SET) { events |= EVENT_ERROR; clear |= hw->errors; }
    /* Equal-priority RX/TX handlers cannot preempt one another. Main only resets
     * events while the requests/channels are disabled and flags cleared. */
    buses[bus].events |= events;
    dma_flag_clear(clear);
}

probe_spi_result probe_spi_poll(probe_spi_bus bus)
{
    const probe_spi_hardware *hw;
    probe_spi_context *ctx;
    uint32_t events;
    if ((unsigned)bus >= PROBE_SPI_COUNT) return PROBE_SPI_INVALID;
    ctx = &buses[bus];
    hw = &hardware[bus];
    if (ctx->result != PROBE_SPI_PENDING) return ctx->result;
    events = ctx->events;
    if ((events & EVENT_ERROR) ||
        spi_i2s_flag_get(hw->spi, SPI_MMERR_FLAG | SPI_I2S_ROERR_FLAG) == SET)
        ctx->result = PROBE_SPI_ERROR;
    else if ((events & (EVENT_RX_DONE | EVENT_TX_DONE)) == (EVENT_RX_DONE | EVENT_TX_DONE) &&
             spi_i2s_flag_get(hw->spi, SPI_I2S_BF_FLAG) == RESET)
        ctx->result = PROBE_SPI_DONE;
    else if (probe_time_us() >= ctx->deadline_us)
        ctx->result = PROBE_SPI_TIMEOUT;
    else
        return PROBE_SPI_PENDING;

    stop_transfer(hw);
    if (ctx->result != PROBE_SPI_DONE) spi_enable(hw->spi, FALSE);
    gpio_bits_set(hw->cs_port, hw->cs_pin);
    if (ctx->result != PROBE_SPI_DONE) configure_spi(hw);
    return ctx->result;
}

const uint8_t *probe_spi_rx(probe_spi_bus bus)
{
    if ((unsigned)bus >= PROBE_SPI_COUNT || buses[bus].result != PROBE_SPI_DONE) return 0;
    return buses[bus].rx;
}
