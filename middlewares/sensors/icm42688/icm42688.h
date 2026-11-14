/**
 * @file icm42688.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef ICM42688_H
#define ICM42688_H

#include "sensor_register.h"

#define ICM42688_ID_REG 0x75U
#define ICM42688_ID_BYTES 1U
#define ICM42688_SNAPSHOT_REG 0x1DU
#define ICM42688_SNAPSHOT_BYTES 14U

/* Stateless chip operations. No MCU, transport, logging or mutable globals.
 * Input buffers contain register payload only, without the SPI command byte. */
uint8_t icm42688_spi_command(uint8_t reg, int read);
int icm42688_identity_valid(const uint8_t *id, unsigned size);
int icm42688_decode_snapshot(const uint8_t *data, unsigned size,
                           int32_t accel[3], int32_t gyro[3]);
/* M2 register-snapshot configuration only; NOT the production FIFO profile.
 * The caller owns transfer progress, wait deadlines and readback failures. */
const sensor_register_setting *icm42688_diagnostic_settings(unsigned *count);

typedef struct {
    uint32_t odr_hz; /* Both accelerometer and gyroscope */
    uint16_t accel_g;
    uint16_t gyro_dps;
} icm42688_config;

typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
    uint16_t timestamp;
    int8_t temperature;
    uint8_t odr_changed;
} icm42688_fifo_sample;

#define ICM42688_FIFO_STATUS_REG 0x2DU
#define ICM42688_POWER_REG 0x4EU
#define ICM42688_BANK_REG 0x76U
#define ICM42688_STATUS_FULL 0x02U
#define ICM42688_FIFO_REG 0x30U
#define ICM42688_FIFO_PACKET_BYTES 16U
#define ICM42688_FIFO_MAX_BYTES 2080U
/* Bank changes are explicit steps. Uses byte count and big-endian data,
 * stop-on-full, absolute 1 us timestamps. Caller discards startup data. */
unsigned icm42688_build_fifo_config(const icm42688_config *config,
                                  sensor_register_setting *steps, unsigned capacity);
/* Accept only the configured six-axis 16-byte ODR-timestamp packet.
 * Empty, other formats, FSYNC and invalid axis values return zero. */
int icm42688_decode_fifo(const uint8_t *data, unsigned size, icm42688_fifo_sample *sample);

#endif
