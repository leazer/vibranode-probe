/**
 * @file probe_spi.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef PROBE_SPI_H
#define PROBE_SPI_H
#include <stdint.h>

typedef enum { PROBE_SPI_ADXL, PROBE_SPI_ICM, PROBE_SPI_COUNT } probe_spi_bus;
typedef enum {
    PROBE_SPI_IDLE, PROBE_SPI_PENDING, PROBE_SPI_DONE,
    PROBE_SPI_TIMEOUT, PROBE_SPI_ERROR, PROBE_SPI_INVALID, PROBE_SPI_BUSY
} probe_spi_result;

#define PROBE_SPI_MAX_BYTES 1025U /* 1 command byte + the 1024 B FIFO-read cap */
/* Main-loop-only API. Each bus owns its DMA buffers; TX is copied on submit.
 * RX remains valid until the next successful submit on that bus. */
void probe_spi_init(void);
probe_spi_result probe_spi_start(probe_spi_bus bus, const uint8_t *tx,
                                 uint16_t size, uint32_t timeout_us);
probe_spi_result probe_spi_poll(probe_spi_bus bus);
const uint8_t *probe_spi_rx(probe_spi_bus bus);
void probe_spi_dma_isr(probe_spi_bus bus);
/* Build dummy clocks directly in the DMA-owned buffer, no large caller stack. */
probe_spi_result probe_spi_read(probe_spi_bus bus, uint8_t command,
                                uint16_t payload_size, uint32_t timeout_us);
#endif
