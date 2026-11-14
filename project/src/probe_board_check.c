/**
 * @file probe_board_check.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <string.h>
#include <math.h>
#include "probe_board_check.h"
#include "probe_acquisition.h"
#include "probe_attitude.h"
#include "probe_calib.h"
#include "probe_led.h"
#include "probe_stack.h"
#include "probe_time.h"
#include "SEGGER_RTT.h"
#define LOG_TAG "BOARD"
#include "vibra_log.h"

enum { CAL_IDLE, CAL_FACE_WAIT, CAL_FACE, CAL_GYRO };
typedef struct {
    probe_acquisition_config config;
    probe_acquisition_status status[2];
    probe_attitude_status attitude;
    probe_calib_record candidate;
    uint64_t report_us, led_until, capture_until, next_sequence;
    uint32_t consumed[2], uncertain[2], period_q16[2];
    uint32_t acq_max_us, attitude_max_us, loop_max_us;
    uint32_t capture_segment, count;
    double sum[6], square[6], temp_sum, faces[6][3];
    uint8_t restart, consume, cal_state, face, dirty;
} board_check_context;
static board_check_context board;
static const char *const face_names[] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};

static void refresh_status(void)
{
    unsigned bus;
    for (bus = 0; bus < 2; ++bus)
        probe_acquisition_get_status((probe_spi_bus)bus, &board.status[bus]);
}

static void stop_capture(void)
{
    board.cal_state = CAL_IDLE;
    probe_attitude_stop();
    probe_acquisition_stop();
}

static void print_calibration(void)
{
    unsigned i;
    LOG_I("CAL candidate flags=%u dirty=%u generation=%u (not applied to raw/AGM)",
          board.candidate.valid_flags, board.dirty, board.candidate.generation);
    for (i = 0; i < 3; ++i)
        LOG_I("CAL axis=%u bias_ug=%d scale_ppm=%d gyro_mdps=%d", i,
              (int)(board.candidate.accel_bias_g[i] * 1000000),
              (int)(board.candidate.accel_scale[i] * 1000000),
              (int)(board.candidate.gyro_bias_dps[i] * 1000));
}

static void help(void)
{
    LOG_I("M4-M8 single-board RTT; no RS485 session; diagnostic abort discards FIFO tail");
    LOG_I("s/r=start/restart x=stop q=consumer toggle; 1=4k/8k 2=250/200 3=4k/16k 4=4k/32k");
    LOG_I("l=LED sweep (12s) c=new ADXL six-face n=capture next face g=ICM gyro static");
    LOG_I("w=save dirty calibration ONLY when stopped; v=reload/discard candidate; p=report h=help");
}

static int ready_to_calibrate(void)
{
    if (!board.consume || board.restart || !board.status[0].running || !board.status[1].running ||
        board.config.adxl.odr_hz != 4000 || board.config.icm.odr_hz != 8000) {
        LOG_W("CAL requires running default 4k/8k configuration and raw consumer");
        return 0;
    }
    return 1;
}

static void begin_window(unsigned state)
{
    memset(board.sum, 0, sizeof(board.sum));
    memset(board.square, 0, sizeof(board.square));
    board.temp_sum = 0;
    board.count = 0;
    board.cal_state = (uint8_t)state;
    board.capture_until = probe_time_us() + 3000000; /* timeout, never a blocking wait */
    LOG_I("CAL collecting 1s of consecutive samples; keep still");
}

static void handle_key(int key)
{
    switch (key) {
    case 'h': help(); return;
    case 'p': board.report_us = 0; print_calibration(); return;
    case 'x': board.restart = 0; stop_capture(); LOG_I("STOP requested"); return;
    case 'q':
        board.consume = !board.consume;
        board.cal_state = CAL_IDLE;
        LOG_I("consumer=%u (off injects queue loss)", board.consume); return;
    case 'l': board.led_until = probe_time_us() + 14000000; return;
    case 'c':
        if (!ready_to_calibrate()) return;
        board.face = 0; board.cal_state = CAL_FACE_WAIT;
        LOG_I("CAL place ADXL +X up, wait until still, then n; order +X -X +Y -Y +Z -Z"); return;
    case 'n':
        if (board.cal_state == CAL_FACE_WAIT && ready_to_calibrate()) begin_window(CAL_FACE);
        return;
    case 'g':
        if (board.cal_state == CAL_IDLE && ready_to_calibrate()) begin_window(CAL_GYRO);
        else LOG_W("finish/cancel face calibration before gyro capture");
        return;
    case 'w':
        if (board.restart || !board.status[0].stopped || !board.status[1].stopped || !board.dirty) {
            LOG_W("SAVE rejected: stop both sensors with x and complete calibration first"); return;
        }
        board.candidate.created_ms = (uint32_t)(probe_time_us() / 1000);
        if (probe_calib_save(&board.candidate)) {
            probe_calib_get(&board.candidate); board.dirty = 0;
            LOG_I("SAVE PASS generation=%u", board.candidate.generation);
        } else LOG_E("SAVE failed; previous record retained");
        return;
    case 'v':
        if (board.restart || !board.status[0].stopped || !board.status[1].stopped) {
            LOG_W("reload requires stopped sensors"); return;
        }
        probe_calib_init(); probe_calib_get(&board.candidate); board.dirty = 0;
        print_calibration(); return;
    case '1': board.config.adxl.odr_hz = 4000; board.config.icm.odr_hz = 8000; break;
    case '2': board.config.adxl.odr_hz = 250; board.config.icm.odr_hz = 200; break;
    case '3': board.config.adxl.odr_hz = 4000; board.config.icm.odr_hz = 16000; break;
    case '4': board.config.adxl.odr_hz = 4000; board.config.icm.odr_hz = 32000; break;
    case 's': case 'r': break;
    default: return;
    }
    stop_capture(); board.restart = 1;
}

static void reject_window(void)
{
    LOG_W("CAL rejected: motion/orientation/gap/timeout; candidate unchanged");
    board.cal_state = board.cal_state == CAL_FACE ? CAL_FACE_WAIT : CAL_IDLE;
}

static void finish_window(void)
{
    double mean[6];
    unsigned i, channels = board.cal_state == CAL_FACE ? 3 : 6;
    probe_calib_record fitted;
    for (i = 0; i < channels; ++i) {
        double variance, limit = board.cal_state == CAL_FACE ? 1048.576 : (i < 3 ? 65.536 : 40.96);
        mean[i] = board.sum[i] / board.count;
        variance = board.square[i] / board.count - mean[i] * mean[i];
        if (!(variance <= limit * limit)) { reject_window(); return; }
    }
    if (board.cal_state == CAL_FACE) {
        unsigned axis = board.face / 2;
        double expected = board.face & 1 ? -52428.8 : 52428.8;
        for (i = 0; i < 3; ++i) {
            if (fabs(mean[i] - (i == axis ? expected : 0)) > 7864.32) { reject_window(); return; }
        }
        memcpy(board.faces[board.face], mean, 3 * sizeof(double));
        LOG_I("CAL face %s accepted samples=%u", face_names[board.face], board.count);
        if (++board.face < 6) {
            board.cal_state = CAL_FACE_WAIT;
            LOG_I("CAL next %s up; wait until still then n", face_names[board.face]); return;
        }
        if (!probe_calib_fit_accel_six_positions((const double (*)[3])board.faces, &fitted)) {
            board.cal_state = CAL_IDLE; LOG_W("CAL six-face fit rejected; restart with c"); return;
        }
        memcpy(board.candidate.accel_bias_g, fitted.accel_bias_g, sizeof(fitted.accel_bias_g));
        memcpy(board.candidate.accel_scale, fitted.accel_scale, sizeof(fitted.accel_scale));
        board.candidate.valid_flags |= PROBE_CALIB_VALID_ACCEL;
    } else {
        double gravity = 0;
        for (i = 0; i < 3; ++i) {
            if (fabs(mean[i]) > 327.68) { reject_window(); return; } /* |mean| <= 5 dps */
            gravity += mean[i + 3] * mean[i + 3];
            mean[i] /= 65.536; /* fixed +/-500 dps */
        }
        if (gravity < 0.85 * 0.85 * 2048 * 2048 || gravity > 1.15 * 1.15 * 2048 * 2048) {
            reject_window(); return;
        }
        probe_calib_fit_gyro_static(mean, &fitted);
        memcpy(board.candidate.gyro_bias_dps, fitted.gyro_bias_dps, sizeof(fitted.gyro_bias_dps));
        board.candidate.temp_c = (float)(25.0 + board.temp_sum / board.count / 2.07);
        board.candidate.valid_flags |= PROBE_CALIB_VALID_GYRO;
    }
    board.cal_state = CAL_IDLE; board.dirty = 1;
    LOG_I("CAL PASS in RAM; x then w to save, v when stopped to discard");
    print_calibration();
}

static void capture_batch(unsigned bus, const probe_raw_batch *batch)
{
    unsigned offset, i, width, target;
    if ((board.cal_state != CAL_FACE || bus != 0) && (board.cal_state != CAL_GYRO || bus != 1)) return;
    if ((batch->flags & PROBE_BATCH_NEAR_FULL_SCALE) || (board.count &&
        (batch->segment != board.capture_segment || batch->first_sequence != board.next_sequence))) {
        reject_window(); return;
    }
    board.capture_segment = batch->segment;
    board.next_sequence = batch->first_sequence + batch->samples;
    width = bus ? ICM42688_FIFO_PACKET_BYTES : ADXL357_FIFO_XYZ_BYTES;
    target = bus ? 8000 : 4000;
    for (offset = 0; offset + width <= batch->size && board.count < target; offset += width) {
        int32_t values[6];
        if (bus) {
            icm42688_fifo_sample sample;
            if (!icm42688_decode_fifo(batch->data + offset, width, &sample)) { reject_window(); return; }
            for (i = 0; i < 3; ++i) { values[i] = sample.gyro[i]; values[i+3] = sample.accel[i]; }
            board.temp_sum += sample.temperature;
        } else if (!adxl357_decode_fifo_xyz(batch->data + offset, width, values)) { reject_window(); return; }
        for (i = 0; i < (bus ? 6U : 3U); ++i) {
            board.sum[i] += values[i]; board.square[i] += (double)values[i] * values[i];
        }
        ++board.count;
    }
    if (board.count == target) finish_window();
}

static void report(void)
{
    unsigned bus;
    probe_attitude_get_status(&board.attitude);
    for (bus = 0; bus < 2; ++bus) {
        const probe_acquisition_status *s = &board.status[bus];
        LOG_I("%s run=%u stop=%u samples=%u consumed=%u seg=%u gap=%u irq=%u",
              bus ? "ICM" : "ADXL", s->running, s->stopped, s->samples, board.consumed[bus],
              s->segment, s->unknown_gaps, s->irq_count);
        LOG_I("%s spi=%u ovr=%u fmt=%u qdrop=%u attdrop=%u fault=%u fifo_max=%u poll_gap=%u",
              bus ? "ICM" : "ADXL", s->spi_errors, s->overflows, s->format_errors, s->queue_drops,
              s->att_drops, s->last_fault, s->fifo_highwater, s->max_poll_gap_us);
        LOG_I("%s time_valid=%u period_q16=%u tick_valid=%u tick_q16=%u uncertain_batches=%u",
              bus ? "ICM" : "ADXL", s->time_valid, board.period_q16[bus], s->tick_valid,
              s->tick_period_q16, board.uncertain[bus]);
        LOG_I("%s a=%d,%d,%d g=%d,%d,%d", bus ? "ICM" : "ADXL",
              s->accel[0], s->accel[1], s->accel[2], s->gyro[0], s->gyro[1], s->gyro[2]);
    }
    LOG_I("ATT in=%u out=%u processed=%u valid=%u decode_drop=%u q30=%d,%d,%d,%d",
          board.attitude.samples_in, board.attitude.resampled, board.attitude.processed,
          board.attitude.quat_valid, board.attitude.dropped_batches,
          board.attitude.quat_q30[0], board.attitude.quat_q30[1], board.attitude.quat_q30[2], board.attitude.quat_q30[3]);
    LOG_I("PERF max_us acq=%u attitude=%u loop=%u stack_unused=%u/%u (since boot)",
          board.acq_max_us, board.attitude_max_us, board.loop_max_us, probe_stack_unused(), probe_stack_capacity());
}

void probe_board_check_init(void)
{
    memset(&board, 0, sizeof(board));
    board.config.adxl.odr_hz = 4000; board.config.adxl.range_g = 10;
    board.config.icm.odr_hz = 8000; board.config.icm.accel_g = 16; board.config.icm.gyro_dps = 500;
    board.restart = board.consume = 1;
    probe_acquisition_init(); probe_attitude_init(); probe_led_init(); probe_calib_init();
    probe_calib_get(&board.candidate);
    help(); print_calibration();
}

void probe_board_check_poll(void)
{
    uint64_t start = probe_time_us(), before, now;
    uint32_t elapsed;
    unsigned bus, state;
    before = probe_time_us(); probe_acquisition_poll();
    elapsed = (uint32_t)(probe_time_us() - before);
    if (elapsed > board.acq_max_us) board.acq_max_us = elapsed;
    refresh_status(); handle_key(SEGGER_RTT_GetKey());
    if (board.restart && board.status[0].stopped && board.status[1].stopped) {
        board.restart = 0;
        if (probe_acquisition_start(&board.config)) {
            memset(board.consumed, 0, sizeof(board.consumed));
            memset(board.uncertain, 0, sizeof(board.uncertain));
            memset(board.period_q16, 0, sizeof(board.period_q16));
            LOG_I("START ADXL=%u ICM=%u attitude_active=%u", board.config.adxl.odr_hz,
                  board.config.icm.odr_hz, probe_attitude_start(&board.config.icm));
        } else LOG_E("START rejected");
    }
    before = probe_time_us(); probe_attitude_poll();
    elapsed = (uint32_t)(probe_time_us() - before);
    if (elapsed > board.attitude_max_us) board.attitude_max_us = elapsed;
    if (board.consume) for (bus = 0; bus < 2; ++bus) {
        const probe_raw_batch *batch = probe_acquisition_peek((probe_spi_bus)bus);
        if (!batch) continue;
        board.consumed[bus] += batch->samples; board.period_q16[bus] = batch->period_q16;
        if (batch->flags & PROBE_BATCH_TIME_UNCERTAIN) ++board.uncertain[bus];
        capture_batch(bus, batch);
        probe_acquisition_release((probe_spi_bus)bus);
    }
    now = probe_time_us(); refresh_status();
    if ((board.cal_state == CAL_FACE || board.cal_state == CAL_GYRO) && now >= board.capture_until) reject_window();
    state = board.status[0].running || board.status[1].running ? 2 : 1;
    if (board.cal_state != CAL_IDLE) state = 3;
    if (board.status[1].stopped && board.status[1].last_fault) state = 4;
    if (board.status[0].stopped && board.status[0].last_fault) state = 5;
    if (now < board.led_until) state = (unsigned)((14000000 - (board.led_until - now)) / 2000000);
    probe_led_update(state);
    if (now >= board.report_us) { board.report_us = now + 1000000; report(); }
    elapsed = (uint32_t)(probe_time_us() - start);
    if (elapsed > board.loop_max_us) board.loop_max_us = elapsed;
}
