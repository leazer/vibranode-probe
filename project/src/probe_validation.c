/**
 * @file probe_validation.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "at32f413_wk_config.h"
#include "adxl357.h"
#include "icm42688.h"
#include "probe_spi.h"
#include "probe_time.h"
#include "probe_validation.h"
#include "SEGGER_RTT.h"
#define LOG_TAG "CHECK"
#include "vibra_log.h"

typedef enum {
    CHECK_ID_START, CHECK_ID_WAIT, CHECK_WRITE_START, CHECK_WRITE_WAIT,
    CHECK_VERIFY_START, CHECK_VERIFY_WAIT, CHECK_SAMPLE_START, CHECK_SAMPLE_WAIT,
    CHECK_FAILED
} check_state;
typedef struct {
    check_state state;
    unsigned setting;
    uint64_t due_us;
    uint32_t samples, errors;
    int32_t accel[3], gyro[3];
    uint8_t id[4];
} sensor_check;
typedef struct {
    uint32_t count;
    uint64_t last_us;
    uint32_t interval_us;
} irq_check;
typedef struct {
    sensor_check sensors[PROBE_SPI_COUNT];
    volatile irq_check irqs[3];
    uint64_t report_due_us;
    uint64_t led_due_us;
    unsigned led_step;
} validation_context;
static validation_context validation;

static void fail_sensor(probe_spi_bus bus, const char *reason, unsigned detail)
{
    sensor_check *s = &validation.sensors[bus];
    ++s->errors;
    s->state = CHECK_FAILED;
    LOG_E("%s FAIL %s detail=%u; press r to retry", bus == PROBE_SPI_ADXL ? "ADXL" : "ICM", reason, detail);
}

static int submit_read(probe_spi_bus bus, uint8_t reg, unsigned length)
{
    uint8_t tx[16] = {0};
    if (length >= sizeof(tx)) { fail_sensor(bus, "read size", length); return 0; }
    tx[0] = bus == PROBE_SPI_ADXL ? adxl357_spi_command(reg, 1) : icm42688_spi_command(reg, 1);
    if (probe_spi_start(bus, tx, (uint16_t)(length + 1), 2000) != PROBE_SPI_PENDING) {
        fail_sensor(bus, "submit read", reg);
        return 0;
    }
    return 1;
}

static void sensor_poll(probe_spi_bus bus, uint64_t now)
{
    sensor_check *s = &validation.sensors[bus];
    unsigned setting_count;
    const sensor_register_setting *settings = bus == PROBE_SPI_ADXL ?
        adxl357_diagnostic_settings(&setting_count) : icm42688_diagnostic_settings(&setting_count);
    const uint8_t *rx;
    probe_spi_result result;
    if (s->state == CHECK_FAILED || now < s->due_us) return;
    if (s->state == CHECK_ID_WAIT || s->state == CHECK_WRITE_WAIT ||
        s->state == CHECK_VERIFY_WAIT || s->state == CHECK_SAMPLE_WAIT) {
        result = probe_spi_poll(bus);
        if (result == PROBE_SPI_PENDING) return;
        if (result != PROBE_SPI_DONE) { fail_sensor(bus, "SPI", result); return; }
    }
    rx = probe_spi_rx(bus);
    switch (s->state) {
    case CHECK_ID_START:
        if (submit_read(bus, bus == PROBE_SPI_ADXL ? ADXL357_ID_REG : ICM42688_ID_REG,
                        bus == PROBE_SPI_ADXL ? ADXL357_ID_BYTES : ICM42688_ID_BYTES))
            s->state = CHECK_ID_WAIT;
        break;
    case CHECK_ID_WAIT:
        memcpy(s->id, rx + 1, bus == PROBE_SPI_ADXL ? ADXL357_ID_BYTES : ICM42688_ID_BYTES);
        if (bus == PROBE_SPI_ADXL) {
            LOG_I("ADXL ID=%02x %02x %02x rev=%02x", s->id[0], s->id[1], s->id[2], s->id[3]);
            if (!adxl357_identity_valid(s->id, ADXL357_ID_BYTES)) { fail_sensor(bus, "identity", s->id[2]); return; }
        } else {
            LOG_I("ICM WHO_AM_I=%02x", s->id[0]);
            if (!icm42688_identity_valid(s->id, ICM42688_ID_BYTES)) { fail_sensor(bus, "identity", s->id[0]); return; }
        }
        s->state = CHECK_WRITE_START;
        break;
    case CHECK_WRITE_START:
        if (s->setting == setting_count) {
            LOG_I("%s CONFIG PASS; diagnostic snapshots only", bus == PROBE_SPI_ADXL ? "ADXL" : "ICM");
            s->state = CHECK_SAMPLE_START;
        } else {
            uint8_t tx[2];
            tx[0] = bus == PROBE_SPI_ADXL ? adxl357_spi_command(settings[s->setting].reg, 0) :
                                                   icm42688_spi_command(settings[s->setting].reg, 0);
            tx[1] = settings[s->setting].value;
            if (probe_spi_start(bus, tx, 2, 2000) == PROBE_SPI_PENDING) s->state = CHECK_WRITE_WAIT;
            else fail_sensor(bus, "submit write", tx[0]);
        }
        break;
    case CHECK_WRITE_WAIT:
        s->due_us = now + settings[s->setting].wait_us;
        if (settings[s->setting].mask) s->state = CHECK_VERIFY_START;
        else { ++s->setting; s->state = CHECK_WRITE_START; }
        break;
    case CHECK_VERIFY_START:
        if (submit_read(bus, settings[s->setting].reg, 1)) s->state = CHECK_VERIFY_WAIT;
        break;
    case CHECK_VERIFY_WAIT:
        if ((rx[1] & settings[s->setting].mask) != (settings[s->setting].value & settings[s->setting].mask)) {
            LOG_E("register %02x expected=%02x got=%02x", settings[s->setting].reg, settings[s->setting].value, rx[1]);
            fail_sensor(bus, "readback", settings[s->setting].reg);
            return;
        }
        ++s->setting;
        s->state = CHECK_WRITE_START;
        break;
    case CHECK_SAMPLE_START:
        /* ADXL: status/FIFO count/temp/XYZ. Status read clears mapped INT1.
         * ICM: register temperature/accel/gyro. These snapshots are not a stream. */
        if (submit_read(bus, bus == PROBE_SPI_ADXL ? ADXL357_SNAPSHOT_REG : ICM42688_SNAPSHOT_REG,
                        bus == PROBE_SPI_ADXL ? ADXL357_SNAPSHOT_BYTES : ICM42688_SNAPSHOT_BYTES))
            s->state = CHECK_SAMPLE_WAIT;
        break;
    case CHECK_SAMPLE_WAIT:
        if (bus == PROBE_SPI_ADXL) {
            if (!adxl357_decode_snapshot(rx + 1, ADXL357_SNAPSHOT_BYTES, s->accel)) {
                fail_sensor(bus, "sample decode", 0);
                return;
            }
        } else {
            if (!icm42688_decode_snapshot(rx + 1, ICM42688_SNAPSHOT_BYTES, s->accel, s->gyro)) {
                fail_sensor(bus, "sample decode", 0);
                return;
            }
        }
        ++s->samples;
        s->due_us = now + 10000;
        s->state = CHECK_SAMPLE_START;
        break;
    default:
        break;
    }
}

void probe_validation_irq(unsigned source)
{
    uint64_t now;
    volatile irq_check *irq;
    if (source >= 3) return;
    now = probe_time_us();
    irq = &validation.irqs[source];
    if (irq->count) irq->interval_us = (uint32_t)(now - irq->last_us);
    irq->last_us = now;
    ++irq->count;
}

static void led_off(void)
{
    gpio_bits_set(LED_R_GPIO_PORT, LED_R_PIN);
    gpio_bits_set(LED_G_GPIO_PORT, LED_G_PIN);
    gpio_bits_set(LED_B_GPIO_PORT, LED_B_PIN);
}

void probe_validation_init(void)
{
    uint32_t mask = __get_PRIMASK();
    uint64_t now = probe_time_us();
    __disable_irq();
    memset(&validation, 0, sizeof(validation));
    validation.sensors[0].due_us = now + 100000;
    validation.sensors[1].due_us = now + 100000;
    validation.report_due_us = now + 1000000;
    exint_flag_clear(EXINT_LINE_0 | EXINT_LINE_1 | EXINT_LINE_10);
    nvic_irq_enable(EXINT0_IRQn, 3, 0);
    nvic_irq_enable(EXINT1_IRQn, 3, 0);
    nvic_irq_enable(EXINT15_10_IRQn, 3, 0);
    __set_PRIMASK(mask);
    led_off();
    LOG_I("HW CHECK M2: SPI1=6MHz SPI2=12MHz; r=retry l=LED test");
    LOG_I("No link handshake/streaming/attitude in this mode");
}

static void led_poll(uint64_t now)
{
    if (!validation.led_step || now < validation.led_due_us) return;
    led_off();
    switch (validation.led_step++) {
    case 1: gpio_bits_reset(LED_R_GPIO_PORT, LED_R_PIN); LOG_I("LED RED"); break;
    case 2: gpio_bits_reset(LED_G_GPIO_PORT, LED_G_PIN); LOG_I("LED GREEN"); break;
    case 3: gpio_bits_reset(LED_B_GPIO_PORT, LED_B_PIN); LOG_I("LED BLUE"); break;
    default: validation.led_step = 0; LOG_I("LED OFF"); break;
    }
    validation.led_due_us = now + 1000000;
}

void probe_validation_poll(void)
{
    uint64_t now = probe_time_us();
    int key = SEGGER_RTT_GetKey();
    if (key == 'r') {
        probe_spi_init();
        probe_validation_init();
        return;
    }
    if (key == 'l') { validation.led_step = 1; validation.led_due_us = now; }
    sensor_poll(PROBE_SPI_ADXL, now);
    sensor_poll(PROBE_SPI_ICM, now);
    led_poll(now);
    if (now >= validation.report_due_us) {
        irq_check snapshot[3];
        uint32_t mask = __get_PRIMASK();
        unsigned i;
        __disable_irq();
        for (i = 0; i < 3; ++i) {
            snapshot[i].count = validation.irqs[i].count;
            snapshot[i].interval_us = validation.irqs[i].interval_us;
            snapshot[i].last_us = validation.irqs[i].last_us;
        }
        __set_PRIMASK(mask);
        validation.report_due_us = now + 1000000;
        LOG_I("time=%u:%u irq=%u/%u/%u dt_us=%u/%u/%u", (unsigned)(now >> 32), (unsigned)now,
              snapshot[0].count, snapshot[1].count, snapshot[2].count,
              snapshot[0].interval_us, snapshot[1].interval_us, snapshot[2].interval_us);
        for (i = 0; i < PROBE_SPI_COUNT; ++i) {
            const sensor_check *s = &validation.sensors[i];
            LOG_I("%s state=%u samples=%u errors=%u a_raw=%d,%d,%d g_raw=%d,%d,%d",
                  i == 0 ? "ADXL" : "ICM", s->state, s->samples, s->errors,
                  s->accel[0], s->accel[1], s->accel[2], s->gyro[0], s->gyro[1], s->gyro[2]);
        }
    }
}
