# @file check_capture_continuity.py
# @author 煲仔卤煮(ZealerLu)
# @date 2026-09-08
#
# @copyright Copyright (c) 2026
#
"""Check same-segment sample continuity in a PC stream capture."""
import struct
import sys
from pathlib import Path
from pc_host import ATTITUDE, Parser, PER_SAMPLE, decode_attitude, decode_stream


def check(path):
    parser = Parser()
    states = {kind: dict(samples=0, frames=0, uncertain=0, gaps=0,
                         segments=set(), next=None) for kind in PER_SAMPLE}
    states[ATTITUDE] = dict(samples=0, frames=0, uncertain=0, gaps=0,
                            segments=set(), next=None)
    with Path(path).open('rb') as source:
        while chunk := source.read(65536):
            for kind, _, _, payload in parser.feed(chunk):
                if kind not in states:
                    continue
                if kind == ATTITUDE:
                    decoded = decode_attitude(payload)
                    records = [decoded['quat_q30']]
                    segment, seq = decoded['segment'], decoded['sequence']
                else:
                    records = decode_stream(kind, payload)
                    segment, seq = struct.unpack_from('<IQ', payload, 4)
                state = states[kind]
                if state['next'] is not None and state['next'] != (segment, seq):
                    state['gaps'] += 1
                state['next'] = (segment, seq + len(records))
                state['segments'].add(segment)
                state['samples'] += len(records)
                state['frames'] += 1
                state['uncertain'] += bool(payload[30] & 1)
    for kind, state in states.items():
        print(f'{Path(path).name} stream=0x{kind:02x}: {state}')
    raw_states = [states[kind] for kind in PER_SAMPLE]
    if (parser.crc_errors or parser.buf or
            any(not s['samples'] or s['gaps'] for s in raw_states) or
            (states[ATTITUDE]['samples'] and states[ATTITUDE]['gaps'])):
        raise ValueError('capture missing data, discontinuous, or corrupt')


if __name__ == '__main__':
    for filename in sys.argv[1:]:
        check(filename)
