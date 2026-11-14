#!/usr/bin/env python3
# @file pc_host.py
# @author 煲仔卤煮(ZealerLu)
# @date 2026-09-07
#
# @copyright Copyright (c) 2026
#
"""PC-side TAB5 role for VibraNode RS-485 bring-up.

Implements the frozen protocol spec (2026-09-07) well enough to run a full
measurement session against the probe firmware: discovery at 1 Mbps, optional
5 Mbps negotiation, config, start, FETCH drain loop, stop.

Requires: pyserial. Examples:
  python tests/pc_host.py --port COM7 --seconds 10 --dump raw.bin
  python tests/pc_host.py --port COM7 --work 1000000 --config slow   # 1 Mbps only
  python tests/pc_host.py --port COM7 --corrupt-crc 7 --drop 11      # fault injection
"""
import argparse
import random
import struct
import sys
import threading
import time

HEADER = b"\xA5\x5A\xC3\x3C"
HELLO, SET_LINK, SET_CONFIG, START, FETCH, STOP, CALIBRATE, SAVE_CAL, GET_STATUS = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09
STREAM_ADXL, STREAM_ICM, TIME_INFO, STATUS, FETCH_DONE, ATTITUDE = 0x10, 0x11, 0x12, 0x13, 0x14, 0x20
MAX_PAYLOAD = 1005
PER_SAMPLE = {STREAM_ADXL: 9, STREAM_ICM: 13}

RESULT_NAMES = {0: "OK", 1: "BUSY", 3: "MALFORMED", 4: "INVALID_PARAM",
                5: "UNSUPPORTED", 6: "NOT_READY", 7: "IN_PROGRESS",
                8: "NOT_IMPLEMENTED", 9: "INTERNAL", 10: "CAL_REJECTED"}


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def build(msg_type, session, seq, payload):
    body = struct.pack("<BHI", msg_type, len(payload), session) + \
        struct.pack("<I", seq) + payload
    crc = crc32c(body)
    return HEADER + body + struct.pack("<I", crc)


class Parser:
    """Incremental frame parser mirroring the firmware rules."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        self.buf.extend(data)
        frames = []
        while True:
            start = self.buf.find(HEADER)
            if start < 0:
                self.buf = self.buf[-3:]
                break
            if start:
                del self.buf[:start]
            if len(self.buf) < 19:
                break
            msg_type, length = struct.unpack_from("<BH", self.buf, 4)
            if length > MAX_PAYLOAD or len(self.buf) < 19 + length:
                if length > MAX_PAYLOAD:
                    del self.buf[0]
                    continue
                break
            total = 19 + length
            frame = bytes(self.buf[:total])
            body = frame[4:-4]
            if crc32c(body) == struct.unpack_from("<I", frame, total - 4)[0]:
                session, seq = struct.unpack_from("<II", frame, 7)
                frames.append((msg_type, session, seq, frame[15:15 + length]))
                del self.buf[:total]
            else:
                self.crc_errors += 1
                del self.buf[0]
        return frames


class Host:
    def __init__(self, args):
        import serial
        self.ser = serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=0.5)
        self.parser = Parser()
        self.session = 0
        self.seq = 0
        self.req = 0
        self.args = args
        self.frames_tx = 0

    def request(self, msg_type, payload, timeout=0.5):
        self.req += 1
        self.seq += 1
        payload = struct.pack('<I', self.req) + payload
        frame = build(msg_type, self.session, self.seq, payload)
        self.frames_tx += 1
        if self.args.corrupt_crc and self.frames_tx % self.args.corrupt_crc == 0:
            frame = frame[:-1] + bytes([frame[-1] ^ 0xFF])
        self.ser.write(frame)
        deadline = time.time() + timeout
        while time.time() < deadline:
            data = self.ser.read(max(1, self.ser.in_waiting))
            if self.args.drop and random.random() < 1.0 / self.args.drop:
                continue  # simulate a lost RX burst
            for msg_type_r, _s, _q, payload_r in self.parser.feed(data):
                # Frozen GET_STATUS snapshot has no request-id/result prefix.
                if msg_type == GET_STATUS and msg_type_r == GET_STATUS and _s == self.session and len(payload_r) == 115:
                    return payload_r
                if msg_type_r == msg_type and payload_r[:4] == struct.pack("<I", self.req):
                    return payload_r
        return None

    def result(self, payload):
        return payload[4] if payload else None

    def hello(self):
        proposed = random.getrandbits(31) | 1
        reply = self.request(HELLO, struct.pack("<I", proposed))
        if reply and reply[4] == 0:
            self.session = proposed
            return True
        return False


def decode_stream(msg_type, payload):
    if msg_type not in PER_SAMPLE or len(payload) < 32:
        raise ValueError('invalid stream header')
    count = struct.unpack_from('<H', payload, 28)[0]
    width = PER_SAMPLE[msg_type]
    if len(payload) != 32 + count * width:
        raise ValueError('stream sample count/length mismatch')
    records = []
    for offset in range(32, len(payload), width):
        if msg_type == STREAM_ICM:
            records.append(struct.unpack_from('<6hb', payload, offset))
        else:
            xyz = []
            for axis in range(3):
                start = offset + axis * 3
                value = int.from_bytes(payload[start:start + 3], 'little') & 0xFFFFF
                xyz.append(value - (1 << 20) if value & (1 << 19) else value)
            records.append(tuple(xyz))
    return records


def decode_attitude(payload):
    if len(payload) != 48:
        raise ValueError('invalid attitude payload length')
    config_id, segment, sequence, time_us, period_q16, count, flags = \
        struct.unpack_from('<IIQQIHH', payload)
    if count != 1 or period_q16 != 5000 << 16:
        raise ValueError('invalid attitude metadata')
    return {'config_id': config_id, 'segment': segment, 'sequence': sequence,
            'time_us': time_us, 'period_q16': period_q16, 'flags': flags,
            'quat_q30': struct.unpack_from('<4i', payload, 32)}


def decode_calibrate_status(payload):
    if len(payload) != 16:
        raise ValueError('invalid CALIBRATE status length')
    request_id, result, state, target, face, completed, valid, dirty, last, count = \
        struct.unpack('<IB7BI', payload)
    return {'request_id': request_id, 'result': result, 'state': state,
            'target': target, 'face': face, 'completed_mask': completed,
            'candidate_valid_flags': valid, 'dirty': dirty,
            'last_result': last, 'sample_count': count}


def decode_calibration_record(payload):
    if len(payload) != 81:
        raise ValueError('invalid SAVE_CAL record length')
    request_id, result, version, valid_flags, _reserved, generation, created_ms = \
        struct.unpack_from('<IBBBHII', payload)
    values = struct.unpack_from('<16f', payload, 17)
    return {'request_id': request_id, 'result': result, 'version': version,
            'valid_flags': valid_flags, 'generation': generation,
            'created_ms': created_ms, 'adxl_accel_bias_g': values[0:3],
            'adxl_accel_scale': values[3:6], 'icm_accel_bias_g': values[6:9],
            'icm_accel_scale': values[9:12], 'gyro_bias_dps': values[12:15],
            'temperature_c': values[15]}


def print_calibration_record(record):
    print(f"[host] CAL v{record['version']} flags=0x{record['valid_flags']:02X} "
          f"generation={record['generation']} created_ms={record['created_ms']}")
    print(f"[host] CAL ADXL bias_g={record['adxl_accel_bias_g']} scale={record['adxl_accel_scale']}")
    print(f"[host] CAL ICM accel bias_g={record['icm_accel_bias_g']} scale={record['icm_accel_scale']}")
    print(f"[host] CAL ICM gyro bias_dps={record['gyro_bias_dps']} temp_c={record['temperature_c']:.3f}")


def write_capture(sink, msg_type, session, seq, payload):
    # Full validated frames preserve stream type and boundaries in mixed dumps.
    decode_stream(msg_type, payload)
    sink.write(build(msg_type, session, seq, payload))


def inspect_capture(path):
    parser = Parser()
    counts = {STREAM_ADXL: 0, STREAM_ICM: 0}
    sums = {STREAM_ADXL: [0.0] * 3, STREAM_ICM: [0.0] * 6}
    squares = {STREAM_ADXL: [0.0] * 3, STREAM_ICM: [0.0] * 6}
    with open(path, 'rb') as stream:
        while chunk := stream.read(65536):
            for kind, _session, _seq, payload in parser.feed(chunk):
                if kind not in PER_SAMPLE:
                    continue
                records = decode_stream(kind, payload)
                counts[kind] += len(records)
                width = 3 if kind == STREAM_ADXL else 6
                for row in records:
                    for i in range(width):
                        value = row[i]
                        sums[kind][i] += value
                        squares[kind][i] += float(value) * value
    if parser.crc_errors or parser.buf or not sum(counts.values()):
        raise ValueError('invalid/truncated capture or legacy payload-only dump; recapture with current pc_host')
    # Scaling assumes the slow calibration config (ADXL +-10 g 20-bit,
    # ICM +-16 g / +-500 dps); counts columns stay valid for any config.
    scales = {STREAM_ADXL: (52428.8,) * 3,
              STREAM_ICM: (2048.0,) * 3 + (65.536,) * 3}
    axes = {STREAM_ADXL: ('X', 'Y', 'Z'),
            STREAM_ICM: ('Ax', 'Ay', 'Az', 'Gx', 'Gy', 'Gz')}
    for kind, name in ((STREAM_ADXL, 'ADXL'), (STREAM_ICM, 'ICM')):
        count = counts[kind]
        if not count:
            continue
        print(f'{name}: samples={count}')
        for i, axis in enumerate(axes[kind]):
            mean = sums[kind][i] / count
            std = (squares[kind][i] / count - mean * mean) ** 0.5
            unit = 'dps' if kind == STREAM_ICM and i >= 3 else 'g'
            scale = scales[kind][i]
            print(f'  {axis:>2}_mean_counts={mean:>12.1f} '
                  f'{axis}_mean={mean / scale:+8.4f} {unit} '
                  f'{axis}_std={std / scale:.4f} {unit}')


def drain_window(host, budget, sink, stats):
    host.req += 1
    host.seq += 1
    host.ser.write(build(FETCH, host.session, host.seq, struct.pack("<II", host.req, budget)))
    deadline = time.time() + 0.2
    done = False
    while time.time() < deadline:
        for msg_type, _s, _q, payload in host.parser.feed(host.ser.read(max(1, host.ser.in_waiting))):
            if msg_type in PER_SAMPLE:
                stats[msg_type]["frames"] += 1
                count = struct.unpack_from("<H", payload, 28)[0]
                stats[msg_type]["samples"] += count
                if payload[30] & 0x01:
                    stats[msg_type]["uncertain"] += 1
                if sink:
                    write_capture(sink, msg_type, _s, _q, payload)
            elif msg_type == ATTITUDE:
                sample = decode_attitude(payload)
                stats[ATTITUDE]['frames'] += 1
                stats[ATTITUDE]['samples'] += 1
                stats[ATTITUDE]['uncertain'] += bool(sample['flags'] & 0x01)
                stats[ATTITUDE]['last'] = sample
                if sink:
                    sink.write(build(msg_type, _s, _q, payload))
            elif msg_type == TIME_INFO:
                stats["time_info"] += 1
            elif msg_type == STATUS:
                stats["status"] += 1
            elif msg_type == FETCH_DONE:
                if payload[:4] == struct.pack("<I", host.req):
                    stats['queued_batches'] = (struct.unpack_from('<H', payload, 11)[0]
                                               + struct.unpack_from('<H', payload, 17)[0])
                    done = True
        if done:
            return True
    return done


def wait_for_position(host, prompt, sink, stats):
    done = threading.Event()
    error = []
    def read_input():
        try:
            input(prompt)
        except BaseException as exc:
            error.append(exc)
        finally:
            done.set()
    threading.Thread(target=read_input, daemon=True).start()
    while not done.is_set():
        drain_window(host, 1600, sink, stats)
    for _ in range(20):
        if drain_window(host, 1600, sink, stats) and stats.get('queued_batches', 0) == 0:
            break
    if error:
        raise error[0]


def run_calibration(host, name, sink, stats, single_face=None):
    targets = {'adxl-accel': 1, 'icm-accel': 2, 'icm-gyro': 3}
    target = targets[name]
    labels = ('+X', '-X', '+Y', '-Y', '+Z', '-Z')
    faces = list(enumerate(labels)) if target != 3 else [(0, 'still')]
    if single_face is not None:
        if target == 3:
            raise ValueError('--face applies only to accelerometer calibration')
        faces = [(single_face, labels[single_face])]
    if target != 3 and single_face is None:
        reply = host.request(CALIBRATE, bytes((3, target, 0, 0)))
        if not reply or reply[4] != 0:
            raise RuntimeError('calibration progress reset rejected')
    for face, label in faces:
        wait_for_position(host,
                          f"[host] Place {name} {label} up and keep still, then press Enter...",
                          sink, stats)
        reply = host.request(CALIBRATE, bytes((1, target, face, 0)))
        status = decode_calibrate_status(reply) if reply else None
        if not status or status['result'] != 7:
            raise RuntimeError(f"capture start failed: {status}")
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            drain_window(host, 1600, sink, stats)
            reply = host.request(CALIBRATE, bytes((0, 0, 0, 0)))
            status = decode_calibrate_status(reply) if reply else None
            if status and status['state'] == 0:
                break
        if not status or status['state'] != 0 or status['last_result'] != 0:
            snapshot = host.request(GET_STATUS, b'')
            if snapshot and len(snapshot) == 115:
                print('[host] CAL failure status: '
                      f"ADXL seg={struct.unpack_from('<I', snapshot, 23)[0]} "
                      f"gap={struct.unpack_from('<I', snapshot, 27)[0]} "
                      f"drop={struct.unpack_from('<I', snapshot, 49)[0]}; "
                      f"ICM seg={struct.unpack_from('<I', snapshot, 65)[0]} "
                      f"gap={struct.unpack_from('<I', snapshot, 69)[0]} "
                      f"spi={struct.unpack_from('<I', snapshot, 79)[0]} "
                      f"ovr={struct.unpack_from('<I', snapshot, 83)[0]} "
                      f"fmt={struct.unpack_from('<I', snapshot, 87)[0]} "
                      f"drop={struct.unpack_from('<I', snapshot, 91)[0]}")
            raise RuntimeError(f"capture rejected or timed out: {status}")
        print(f"[host] CAL {label} accepted: samples={status['sample_count']} "
              f"completed=0x{status['completed_mask']:02X}")


def selftest():
    frame_a = HEADER + struct.pack("<BHI", HELLO, 8, 0) +         struct.pack("<I", 0) + bytes.fromhex("0100000044332211") +         struct.pack("<I", 0x2F6BF0BC)
    body = frame_a[4:-4]
    assert crc32c(body) == struct.unpack_from("<I", frame_a, 23)[0], "CRC-32C mismatch"
    assert crc32c(b"123456789") == 0xE3069283, "CRC-32C check vector failed"
    frames = Parser().feed(frame_a)
    assert frames and frames[0][0] == HELLO and frames[0][1] == 0
    assert frames[0][3] == bytes.fromhex("0100000044332211")
    print("selftest OK")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="", help="serial port, e.g. COM7")
    ap.add_argument("--baud", type=int, default=1000000, help="discovery rate")
    ap.add_argument("--work", type=int, default=5000000, help="rate to negotiate")
    ap.add_argument("--config", choices=["default", "slow"])
    ap.add_argument("--seconds", type=float, default=10)
    ap.add_argument("--dump", help="file for complete stream frames (type, metadata, raw data, CRC)")
    ap.add_argument("--inspect", help="inspect a frame capture without serial hardware")
    ap.add_argument("--calibrate", choices=["adxl-accel", "icm-accel", "icm-gyro"],
                    help="run an interactive nonblocking calibration sequence")
    ap.add_argument("--face", type=int, choices=range(6),
                    help="capture one accelerometer face (0..5 = +X,-X,+Y,-Y,+Z,-Z)")
    ap.add_argument("--save-cal", action="store_true", help="save a successful calibration after STOP")
    ap.add_argument("--read-cal", action="store_true", help="read and print the current calibration record")
    ap.add_argument("--corrupt-crc", type=int, default=0, help="corrupt every Nth TX frame")
    ap.add_argument("--drop", type=int, default=0, help="drop roughly 1-in-N RX bursts")
    ap.add_argument("--budget", type=int, default=1600,
                    help="FETCH byte budget per window (TAB5 uses 16384)")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if args.inspect:
        return inspect_capture(args.inspect)
    if not args.port:
        ap.error("--port is required unless --selftest")
    if args.face is not None and not args.calibrate:
        ap.error('--face requires --calibrate')

    host = Host(args)
    print(f"[host] discovering at {args.baud} ...")
    for _ in range(5):
        if host.hello():
            break
        time.sleep(1.0)
    else:
        sys.exit("FAIL: no HELLO reply")
    print(f"[host] session 0x{host.session:08X} bound")

    if args.work != args.baud:
        reply = host.request(SET_LINK, struct.pack("<I", args.work))
        if reply and reply[4] == 0:
            time.sleep(0.15)  # quiet window; both sides switch
            print(f'[host] setting PC serial baud to {args.work}', flush=True)
            host.ser.baudrate = args.work
            time.sleep(0.05)
            confirm = host.request(GET_STATUS, b'')
            if confirm and len(confirm) == 115:
                print(f"[host] link confirmed at {args.work}")
            else:
                print("[host] new rate failed; falling back")
                host.ser.baudrate = args.baud
                if not host.hello():
                    sys.exit("FAIL: fallback HELLO")
        else:
            print("[host] SET_LINK rejected; staying at base rate")

    if args.read_cal:
        reply = host.request(SAVE_CAL, bytes((0, 0, 0, 0)))
        if not reply:
            sys.exit('FAIL: no SAVE_CAL read reply')
        print_calibration_record(decode_calibration_record(reply))
        if not args.calibrate:
            return 0

    config_name = args.config or ('slow' if args.calibrate else 'default')
    if config_name == "default":
        streams = bytes([0x01, 4, 0, 0, 0x02, 5, 0x13, 0])  # ADXL 4k +-10g, ICM 8k 16g 500dps
    else:
        streams = bytes([0x01, 0, 0, 0, 0x02, 0, 0x13, 0])  # ADXL 250, ICM 200
    config_id = random.getrandbits(31) | 1
    reply = host.request(SET_CONFIG, struct.pack("<I", config_id) + bytes([2]) + streams)
    if not reply or reply[4] != 0:
        sys.exit(f"FAIL: SET_CONFIG -> {RESULT_NAMES.get(host.result(reply), host.result(reply))}")
    print("[host] config accepted")

    reply = host.request(START, b'')
    if not reply or reply[4] != 0:
        sys.exit(f"FAIL: START -> {RESULT_NAMES.get(host.result(reply), host.result(reply))}")
    print("[host] started; draining windows ...")

    # Drain immediately; sensor settling is handled by the firmware.
    stats = {STREAM_ADXL: {"frames": 0, "samples": 0, "uncertain": 0},
             STREAM_ICM: {"frames": 0, "samples": 0, "uncertain": 0},
             ATTITUDE: {"frames": 0, "samples": 0, "uncertain": 0, "last": None},
             "time_info": 0, "status": 0, "crc": 0}
    sink = open(args.dump, "wb") if args.dump else None
    started = time.time()
    if args.calibrate:
        run_calibration(host, args.calibrate, sink, stats, args.face)
    else:
        while time.time() - started < args.seconds:
            if not drain_window(host, args.budget, sink, stats):
                stats["crc"] = host.parser.crc_errors
    reply = host.request(STOP, b'')
    boundary = "n/a"
    if reply and reply[4] == 0:
        boundary = f"ADXL seq {struct.unpack_from('<Q', reply, 9)[0]}, " \
                   f"ICM seq {struct.unpack_from('<Q', reply, 21)[0]}"
    drain_deadline = time.monotonic() + 2.0
    while time.monotonic() < drain_deadline:
        if drain_window(host, 1600, sink, stats) and stats.get('queued_batches') == 0:
            break
    else:
        print('[host] WARNING: final drain did not complete')
    if args.calibrate:
        action = 1 if args.save_cal else 0
        reply = host.request(SAVE_CAL, bytes((action, 0, 0, 0)))
        if not reply:
            sys.exit('FAIL: no SAVE_CAL reply')
        record = decode_calibration_record(reply)
        if args.save_cal and record['result'] != 0:
            sys.exit(f"FAIL: SAVE_CAL -> {RESULT_NAMES.get(record['result'], record['result'])}")
        print_calibration_record(record)
    if sink:
        sink.close()
    stats['crc'] = host.parser.crc_errors

    elapsed = time.time() - started
    print(f"[host] STOP boundary: {boundary}")
    for stream, name in ((STREAM_ADXL, "ADXL"), (STREAM_ICM, "ICM")):
        s = stats[stream]
        rate = s["samples"] / elapsed if elapsed else 0
        print(f"[host] {name}: samples={s['samples']} ({rate:.0f}/s) frames={s['frames']} "
              f"time_uncertain_frames={s['uncertain']}")
    attitude = stats[ATTITUDE]
    print(f"[host] ATTITUDE: samples={attitude['samples']} frames={attitude['frames']} "
          f"time_uncertain_frames={attitude['uncertain']}"
          + (f" last_q30={attitude['last']['quat_q30']}" if attitude['last'] else ""))
    print(f"[host] TIME_INFO={stats['time_info']} STATUS={stats['status']} "
          f"host_crc_errors={stats['crc']}")


if __name__ == "__main__":
    main()
