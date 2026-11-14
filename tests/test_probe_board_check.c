/**
 * @file test_probe_board_check.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include "probe_acquisition.h"
#include "probe_attitude.h"
#include "probe_calib.h"
#include "at32f413_flash.h"

void probe_board_check_init(void);
void probe_board_check_poll(void);
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
static uint64_t now;
static int key = -1, starts, att_starts, saves, led, queued, queued_bus;
static probe_acquisition_status acq[2];
static probe_raw_batch batch;
static probe_calib_record saved;
uint64_t probe_time_us(void) { return now; }
unsigned probe_stack_unused(void) { return 1024; }
unsigned probe_stack_capacity(void) { return 2048; }
int SEGGER_RTT_GetKey(void) { int k = key; key = -1; return k; }
void test_log_info(const char *s, ...) { (void)s; }
void test_log_error(const char *s, ...) { (void)s; }
void probe_acquisition_init(void) { memset(acq, 0, sizeof(acq)); acq[0].stopped = acq[1].stopped = 1; }
int probe_acquisition_start(const probe_acquisition_config *c) {
    (void)c; ++starts; acq[0].stopped = acq[1].stopped = 0;
    acq[0].running = acq[1].running = 1; return 1;
}
void probe_acquisition_stop(void) { /* completion is asynchronous */ }
void probe_acquisition_poll(void) { now += 2; }
void probe_acquisition_get_status(probe_spi_bus b, probe_acquisition_status *s) { *s = acq[b]; }
const probe_raw_batch *probe_acquisition_peek(probe_spi_bus b) { return b == queued_bus && queued ? &batch : 0; }
void probe_acquisition_release(probe_spi_bus b) { (void)b; queued = 0; }
void probe_attitude_init(void) { }
int probe_attitude_start(const icm42688_config *c) { if (c->odr_hz == 8000) ++att_starts; return c->odr_hz == 8000; }
void probe_attitude_stop(void) { }
void probe_attitude_poll(void) { now += 3; }
void probe_attitude_get_status(probe_attitude_status *s) { memset(s, 0, sizeof(*s)); }
void probe_led_init(void) { led = 0; }
void probe_led_update(unsigned s) { led = s; }
flash_status_type flash_sector_erase(uint32_t a) { ++saves; memset((void *)a, 255, 2048); return FLASH_OPERATE_DONE; }
flash_status_type flash_word_program(uint32_t a, uint32_t v) { memcpy((void *)a, &v, 4); return FLASH_OPERATE_DONE; }
void flash_unlock(void) { }
void flash_lock(void) { }
static void press(int c) { key = c; probe_board_check_poll(); }
static void feed_face(unsigned face) {
    unsigned i, axis;
    queued_bus = 0;
    memset(&batch, 0, sizeof(batch)); batch.segment = 1; batch.samples = 32; batch.size = 288;
    for (i = 0; i < 32; ++i) for (axis = 0; axis < 3; ++axis) {
        int32_t value = axis == face / 2 ? ((face & 1) ? -52429 : 52429) : 0;
        uint32_t word = ((uint32_t)value & 0xfffffU) << 4;
        unsigned p = i * 9 + axis * 3;
        batch.data[p] = (uint8_t)(word >> 16); batch.data[p+1] = (uint8_t)(word >> 8);
        batch.data[p+2] = (uint8_t)(word | (axis == 0 ? 1 : 0));
    }
    for (i = 0; i < 125; ++i) {
        batch.first_sequence = i * 32; queued = 1; now += 8000; probe_board_check_poll();
    }
}
static void feed_gyro(int motion) {
    unsigned i, j;
    queued_bus = 1;
    memset(&batch, 0, sizeof(batch)); batch.segment = 1; batch.samples = 32; batch.size = 512;
    for (i = 0; i < 32; ++i) {
        int16_t gyro = motion ? (i & 1 ? 200 : -200) : 33;
        unsigned p = i * 16;
        batch.data[p] = 0x68; batch.data[p+5] = 8; /* +1 g Z */
        batch.data[p+7] = (uint8_t)((uint16_t)gyro >> 8); batch.data[p+8] = (uint8_t)gyro;
    }
    for (j = 0; j < 250; ++j) {
        batch.first_sequence = j * 32; queued = 1; now += 4000; probe_board_check_poll();
    }
}
int test_main(void) {
    unsigned face;
    memset((void *)0x0803f000, 255, 4096);
    probe_board_check_init(); probe_board_check_poll();
    CHECK(starts == 1 && att_starts == 1 && saves == 0);
    press('w'); CHECK(saves == 0); /* no implicit/default flash write */
    press('r'); CHECK(starts == 1); /* wait for both stop completions */
    acq[0].stopped = 1; probe_board_check_poll(); CHECK(starts == 1);
    acq[1].stopped = 1; probe_board_check_poll(); CHECK(starts == 2);
    press('c');
    press('n'); feed_face(1); /* wrong orientation must not advance +X */
    press('n'); now += 4000000; probe_board_check_poll(); /* timeout must allow retry */
    for (face = 0; face < 6; ++face) { press('n'); feed_face(face); }
    press('w'); CHECK(saves == 0); /* still running */
    press('x');
    acq[0].running = acq[1].running = 0; acq[0].stopped = acq[1].stopped = 1;
    probe_board_check_poll(); CHECK(led == 1);
    press('w'); CHECK(saves == 1);
    probe_calib_init(); probe_calib_get(&saved);
    CHECK(saved.valid_flags & PROBE_CALIB_VALID_ACCEL);
    CHECK(saved.accel_scale[0] > 0.999f && saved.accel_scale[0] < 1.001f);
    press('w'); CHECK(saves == 1); /* repeated command does not wear flash */
    press('s'); press('g'); feed_gyro(1);
    press('x'); acq[0].running = acq[1].running = 0; acq[0].stopped = acq[1].stopped = 1;
    press('w'); CHECK(saves == 1); /* moving gyro is rejected */
    press('s'); press('g'); feed_gyro(0);
    press('x'); acq[0].running = acq[1].running = 0; acq[0].stopped = acq[1].stopped = 1;
    press('w'); CHECK(saves == 2);
    probe_calib_init(); probe_calib_get(&saved);
    CHECK(saved.valid_flags == (PROBE_CALIB_VALID_ACCEL | PROBE_CALIB_VALID_GYRO));
    CHECK(saved.gyro_bias_dps[0] > 0.50f && saved.gyro_bias_dps[0] < 0.51f);
    press('l'); CHECK(led == 0);
    now += 12000000; probe_board_check_poll(); CHECK(led == 6); /* last sweep slot: no-host white blink */
    now += 2000000; probe_board_check_poll(); CHECK(led == 1);  /* sweep done, live arbitration again */
    press('2'); probe_board_check_poll(); CHECK(att_starts == 4);
    return 0;
}
