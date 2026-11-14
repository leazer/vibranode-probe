/**
 * @file probe_dma.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-06
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "probe_dma.h"
#include "at32f413.h"

void probe_dma_init(void)
{
    dma_init_type config;
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    dma_reset(DMA1_CHANNEL1);
    dma_reset(DMA1_CHANNEL2);
    dma_reset(DMA1_CHANNEL3);
    dma_reset(DMA1_CHANNEL4);
    dma_flexible_config(DMA1, FLEX_CHANNEL1, DMA_FLEXIBLE_SPI1_RX);
    dma_flexible_config(DMA1, FLEX_CHANNEL2, DMA_FLEXIBLE_SPI1_TX);
    dma_flexible_config(DMA1, FLEX_CHANNEL3, DMA_FLEXIBLE_SPI2_RX);
    dma_flexible_config(DMA1, FLEX_CHANNEL4, DMA_FLEXIBLE_SPI2_TX);

    /* Zero-sized inactive channels. Transfer owners set addresses/counts later. */
    dma_default_para_init(&config);
    config.priority = DMA_PRIORITY_VERY_HIGH;
    dma_init(DMA1_CHANNEL1, &config);
    dma_init(DMA1_CHANNEL3, &config);
    config.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA1_CHANNEL2, &config);
    dma_init(DMA1_CHANNEL4, &config);
}
