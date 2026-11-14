# @file run_probe_time.py
# @author 煲仔卤煮(ZealerLu)
# @date 2026-09-06
#
# @copyright Copyright (c) 2026
#
"""Run the actual C time component against fake registers on emulated Cortex-M4.

Requires unicorn==2.1.4 and pyelftools==0.33 (or compatible), ARMCC 5.
Set PYTHONPATH to the dependency installation if not installed globally.
"""
from pathlib import Path
import subprocess
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

root = Path(__file__).resolve().parents[1]
build = root / "tests/build"
build.mkdir(exist_ok=True)
compiler = Path("C:/Keil_v5/ARM/ARMCC/Bin")
objects = []
for source in [root / "tests/test_probe_time.c", root / "project/src/probe_time.c"]:
    obj = build / (source.stem + ".o")
    subprocess.run([str(compiler / "armcc.exe"), "--cpu=Cortex-M4", "--c99", "-O1",
                    "-I" + str(root / "tests/stubs"), "-I" + str(root / "project/inc"),
                    "-c", str(source), "-o", str(obj)], check=True)
    objects.append(str(obj))
target = build / "time-test.axf"
subprocess.run([str(compiler / "armlink.exe"), *objects, "--entry=test_main",
                "--ro-base=0x10000", "--rw-base=0x20000000", "-o", str(target)], check=True)
machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
machine.mem_map(0x10000, 0x10000)
machine.mem_map(0x20000000, 0x20000)
with target.open("rb") as stream:
    elf = ELFFile(stream)
    for section in elf.iter_sections():
        if section["sh_flags"] & 2 and section["sh_type"] != "SHT_NOBITS":
            machine.mem_write(section["sh_addr"], section.data())
    entry = elf.get_section_by_name(".symtab").get_symbol_by_name("test_main")[0]["st_value"]
machine.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
machine.reg_write(UC_ARM_REG_LR, 0x1F001)
machine.emu_start(entry | 1, 0x1F000, count=100000)
if machine.reg_read(UC_ARM_REG_PC) != 0x1F000:
    raise SystemExit("FAIL: test did not return within instruction budget")
result = machine.reg_read(UC_ARM_REG_R0)
if result:
    raise SystemExit(f"FAIL: test_probe_time.c line {result}")
print("PASS: initialization, pending overflow, ISR accounting, read-time rollover, PRIMASK preservation")
