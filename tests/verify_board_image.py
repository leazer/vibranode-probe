# @file verify_board_image.py
# @author 煲仔卤煮(ZealerLu)
# @date 2026-09-07
#
# @copyright Copyright (c) 2026
#
"""Execute C startup and application stack painting from the actual AXF.

Skips hardware SystemInit, runs the real C runtime, stops at board clock init.
"""
from pathlib import Path
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

root = Path(__file__).resolve().parents[1]
machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
machine.mem_map(0x08000000, 0x40000)
machine.mem_map(0x20000000, 0x10000)  # full 64 KB SRAM (EOPB0 0xFA layout)
machine.mem_map(0xE0000000, 0x100000)
# Provisioned state: USD eopb0 (0x1FFFF810) already selects 64 KB SRAM, so the
# early boot guard returns without touching flash.
machine.mem_map(0x1FFFF800, 0x800)
machine.mem_write(0x1FFFF810, b'\xFA\x00')
with (root / 'project/MDK_V5/objects/VibraNode.axf').open('rb') as stream:
    elf = ELFFile(stream)
    symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    for segment in elf.iter_segments():
        if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
            machine.mem_write(segment['p_paddr'], segment.data())
    # All code/RO sections must stop before the calibration sectors.
    for section in elf.iter_sections():
        if section['sh_flags'] & 2 and 0x08000000 <= section['sh_addr'] < 0x08040000:
            assert section['sh_addr'] + section['sh_size'] <= 0x0803F000

base, limit = symbols['__probe_stack_base'], symbols['__probe_stack_limit']
assert limit - base == 2048
machine.reg_write(UC_ARM_REG_SP, limit)
def skip_system_init(cpu, address, size, data):
    if address == symbols['SystemInit'] & ~1:
        cpu.reg_write(UC_ARM_REG_PC, cpu.reg_read(UC_ARM_REG_LR))

machine.hook_add(UC_HOOK_CODE, skip_system_init)
# Execute the real C runtime too: its ZI clear used to erase the watermark.
machine.emu_start(symbols['Reset_Handler'] | 1, symbols['wk_system_clock_config'] & ~1, count=100000)
assert machine.reg_read(UC_ARM_REG_PC) == symbols['wk_system_clock_config'] & ~1
assert machine.mem_read(base, 1024) == b'\xa5' * 1024, 'C runtime erased the watermark before application startup'

# Stack probes reimplemented host-side: the firmware never calls
# probe_stack_capacity/probe_stack_unused, so armlink eliminates those
# functions from the production image.
assert limit - base == 2048
machine.mem_write(base + 1024, b'\x00' * 4)
word = 0
while base + 4 * word < limit:
    if machine.mem_read(base + 4 * word, 4) != b'\xa5\xa5\xa5\xa5':
        break
    word += 1
assert 4 * word == 1024, 'watermark scan stopped at the erased word'
print('PASS: actual image C startup preserves application watermark; overwrite detected; calibration flash excluded')
