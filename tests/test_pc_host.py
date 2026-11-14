# @file test_pc_host.py
# @author 煲仔卤煮(ZealerLu)
# @date 2026-09-07
#
# @copyright Copyright (c) 2026
#
import io
import struct
import unittest
import time
from unittest import mock
from types import SimpleNamespace
import pc_host as host


class HostTests(unittest.TestCase):
    def test_position_prompt_keeps_fetching_until_enter(self):
        calls = []
        def delayed_input(_prompt):
            time.sleep(0.12)
            return ''
        with mock.patch('builtins.input', side_effect=delayed_input), \
             mock.patch.object(host, 'drain_window', side_effect=lambda *_: calls.append(1) or True):
            host.wait_for_position(object(), 'ready', None, {})
        self.assertGreaterEqual(len(calls), 2)

    def test_request_prepends_id_and_matches_reply(self):
        class Serial:
            in_waiting = 4096
            def write(self, data):
                self.frame = host.Parser().feed(data)[0]
            def read(self, size):
                kind, session, seq, payload = self.frame
                return host.build(kind, session, seq, payload[:4] + b'\x00')
        client = host.Host.__new__(host.Host)
        client.ser = Serial()
        client.parser = host.Parser()
        client.session = 123
        client.req = client.seq = client.frames_tx = 0
        client.args = SimpleNamespace(corrupt_crc=0, drop=0)
        body = struct.pack('<I', 987) + bytes([2, 1, 0, 0, 0, 2, 0, 0x13, 0])
        reply = client.request(host.SET_CONFIG, body, timeout=0.01)
        self.assertEqual(client.ser.frame[3], struct.pack('<I', 1) + body)
        self.assertEqual(reply, struct.pack('<IB', 1, 0))
        reply = client.request(host.START, b'', timeout=0.01)
        self.assertEqual(client.ser.frame[3], struct.pack('<I', 2))
        self.assertEqual(reply, struct.pack('<IB', 2, 0))

    def test_mixed_capture_retains_type_and_signed_samples(self):
        # ADXL wire samples are little-endian 20-bit values in three bytes.
        adxl = struct.pack('<IIQQIHH', 1, 2, 3, 4, 5, 1, 0) + bytes.fromhex('ffff07 000008 ffff0f')
        icm = struct.pack('<IIQQIHH', 1, 2, 3, 4, 5, 1, 0) + struct.pack('<6hb', 1, -2, 3, -4, 5, -6, -7)
        sink = io.BytesIO()
        host.write_capture(sink, host.STREAM_ADXL, 123, 10, adxl)
        host.write_capture(sink, host.STREAM_ICM, 123, 11, icm)
        frames = host.Parser().feed(sink.getvalue())
        self.assertEqual([f[0] for f in frames], [0x10, 0x11])
        self.assertEqual(host.decode_stream(frames[0][0], frames[0][3]), [(524287, -524288, -1)])
        self.assertEqual(host.decode_stream(frames[1][0], frames[1][3]), [(1, -2, 3, -4, 5, -6, -7)])

    def test_bad_stream_length_is_rejected(self):
        with self.assertRaises(ValueError):
            host.decode_stream(0x10, bytes(31))
        payload = struct.pack('<IIQQIHH', 1, 2, 3, 4, 5, 2, 0) + bytes(9)
        with self.assertRaises(ValueError):
            host.decode_stream(0x10, payload)

    def test_attitude_payload_decodes_q30_and_metadata(self):
        payload = struct.pack('<IIQQIHH4i', 9, 3, 7, 1234567,
                              5000 << 16, 1, 1,
                              1 << 30, 0, -(1 << 29), 1 << 28)
        decoded = host.decode_attitude(payload)
        self.assertEqual(decoded['config_id'], 9)
        self.assertEqual(decoded['segment'], 3)
        self.assertEqual(decoded['sequence'], 7)
        self.assertEqual(decoded['time_us'], 1234567)
        self.assertEqual(decoded['flags'], 1)
        self.assertEqual(decoded['quat_q30'], (1 << 30, 0, -(1 << 29), 1 << 28))
        with self.assertRaises(ValueError):
            host.decode_attitude(payload[:-1])

    def test_calibration_status_and_record_decode(self):
        status = struct.pack('<IB7B I', 4, 0, 1, 2, 3, 0x0F, 7, 1, 10, 200)
        decoded = host.decode_calibrate_status(status)
        self.assertEqual((decoded['target'], decoded['face'], decoded['completed_mask']),
                         (2, 3, 0x0F))
        values = [0.1, 0.2, 0.3, 1.0, 0.99, 1.01,
                  -0.1, -0.2, -0.3, 1.02, 0.98, 1.0,
                  0.5, -0.25, 0.125, 26.5]
        record = struct.pack('<IBBBHII16f', 5, 0, 3, 7, 0, 9, 1234, *values)
        decoded = host.decode_calibration_record(record)
        self.assertEqual(decoded['version'], 3)
        self.assertEqual(decoded['valid_flags'], 7)
        self.assertEqual(decoded['generation'], 9)
        self.assertAlmostEqual(decoded['icm_accel_bias_g'][1], -0.2, places=5)
        self.assertAlmostEqual(decoded['temperature_c'], 26.5, places=5)


if __name__ == '__main__':
    unittest.main()
