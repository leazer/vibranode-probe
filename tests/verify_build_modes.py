# @file verify_build_modes.py
# @author 煲仔卤煮(ZealerLu)
# @date 2026-09-07
#
# @copyright Copyright (c) 2026
#
"""Rebuild modes 0..3; leave the original project's default USB CDC image last.

Temporary projects stay beside the original so relative include paths resolve.
Never downloads to hardware. Requires Keil ARMCC5 at its configured location.
"""
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parents[1]
directory = root / 'project/MDK_V5'
project = directory / 'VibraNode.uvprojx'
source = project.read_text(encoding='utf-8')
for mode in (1, 2, 3, 0):
    temporary = directory / f'.verify-mode{mode}.uvprojx'
    if temporary.exists():
        raise RuntimeError(f'remove stale temporary project after inspection: {temporary}')
    log = directory / f'verify-mode{mode}.log'
    log.unlink(missing_ok=True)
    try:
        if mode != 0:
            text = source.replace('<Define>AT32F413CCU7, USE_STDPERIPH_DRIVER</Define>',
                                  f'<Define>AT32F413CCU7, USE_STDPERIPH_DRIVER, PROBE_HARDWARE_VALIDATION={mode}</Define>', 1)
            assert text != source
            temporary.write_text(text, encoding='utf-8')
        subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-j0', '-r', str(temporary if mode != 0 else project),
                        '-t', 'VibraNode', '-o', str(log)], check=True)
        result = log.read_text(errors='replace')
        if '0 Error(s), 0 Warning(s)' not in result:
            raise RuntimeError(result)
        size = re.search(r'Program Size:.*', result)
        assert size
        print(f'Mode {mode}: {size.group(0).strip()}; 0 errors, 0 warnings', flush=True)
    finally:
        temporary.unlink(missing_ok=True)
