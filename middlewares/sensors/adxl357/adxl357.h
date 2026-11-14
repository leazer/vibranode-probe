/**
 * @file adxl357.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef ADXL357_H
#define ADXL357_H

#include "sensor_register.h"

#define ADXL357_ID_REG 0x00U
#define ADXL357_ID_BYTES 4U
#define ADXL357_SNAPSHOT_REG 0x04U
#define ADXL357_SNAPSHOT_BYTES 13U

/* Stateless chip operations. No MCU, transport, logging or mutable globals.
 * Input buffers contain register payload only, without the SPI command byte. */
uint8_t adxl357_spi_command(uint8_t reg, int read);
int adxl357_identity_valid(const uint8_t *id, unsigned size);
int adxl357_decode_snapshot(const uint8_t *data, unsigned size,
                           int32_t accel[3]);
/* M2 register-snapshot configuration only; NOT the production FIFO profile.
 * The caller owns transfer progress, wait deadlines and readback failures. */
const sensor_register_setting *adxl357_diagnostic_settings(unsigned *count);

typedef struct {
    uint32_t odr_hz;
    uint16_t range_g;
    uint8_t hpf; /* 0: off, 1..6: hardware HPF codes */
} adxl357_config;

#define ADXL357_POWER_REG 0x2DU
#define ADXL357_STATUS_OVERFLOW 0x04U
#define ADXL357_FIFO_REG 0x11U
#define ADXL357_FIFO_XYZ_BYTES 9U
#define ADXL357_FIFO_MAX_ENTRIES 96U
/* Returns step count, or zero without modifying output for invalid input.
 * Apply only to a stopped device. Last step enables measurement; caller
 * drains/discards startup data for at least 60 ms with HPF off. */
unsigned adxl357_build_fifo_config(const adxl357_config *config,
                                 sensor_register_setting *steps, unsigned capacity);
int adxl357_decode_fifo_xyz(const uint8_t *data, unsigned size, int32_t accel[3]);

#endif
