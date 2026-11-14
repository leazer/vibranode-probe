# @file run_probe_agm.py
# @author 煲仔卤煮(ZealerLu)
# @date 2026-09-06
#
# @copyright Copyright (c) 2026
#
"""Link TDK's GCC archive with ARMCC 5 and run its exact reference vector.
Requires unicorn and pyelftools, as run_probe_time.py does.
This is instruction emulation, not hardware timing or sensor validation.
"""
from pathlib import Path
import subprocess
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC, UC_CPU_ARM_CORTEX_M4

root = Path(__file__).resolve().parents[1]
package = root / "middlewares/agm"
vector_dir = root / "tests/agm"
build = root / "tests/build"
build.mkdir(exist_ok=True)
compiler = Path("C:/Keil_v5/ARM/ARMCC/Bin")
obj = build / "agm-test.o"
subprocess.run([str(compiler / "armcc.exe"), "--cpu=Cortex-M4.fp", "--apcs=/hardfp",
                "--c99", "--wchar32", "-O1", "-I" + str(package / "include"),
                "-I" + str(vector_dir),
                "-c", str(root / "tests/test_probe_agm.c"), "-o", str(obj)], check=True)
target = build / "agm-test.axf"
subprocess.run([str(compiler / "armlink.exe"), str(obj),
                str(package / "lib/libInvnAlgoAGM.a"), "--entry=test_main",
                "--ro-base=0x10000", "--rw-base=0x20000000", "--map", "--info=sizes",
                "--list=" + str(build / "agm-test.map"), "-o", str(target)], check=True)
machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
machine.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
machine.mem_map(0x10000, 0x400000)
machine.mem_map(0x20000000, 0x100000)
machine.mem_map(0xE000E000, 0x2000)
machine.mem_write(0xE000ED88, (0xF << 20).to_bytes(4, "little"))
with target.open("rb") as stream:
    elf = ELFFile(stream)
    for section in elf.iter_sections():
        if section["sh_flags"] & 2 and section["sh_type"] != "SHT_NOBITS":
            machine.mem_write(section["sh_addr"], section.data())
    entry = elf.get_section_by_name(".symtab").get_symbol_by_name("test_main")[0]["st_value"]
machine.reg_write(UC_ARM_REG_SP, 0x200FFFF0)
machine.reg_write(UC_ARM_REG_LR, 0x400001)
machine.emu_start(entry | 1, 0x400000, count=300000000)
if machine.reg_read(UC_ARM_REG_PC) != 0x400000:
    raise SystemExit("FAIL: test did not return within instruction budget")
result = machine.reg_read(UC_ARM_REG_R0)
if result:
    raise SystemExit(f"FAIL: reference row {result} (0xFFFFFFFF means init failure)")
print("PASS: full TDK reference vector, quaternion and all three sensor bias outputs match exactly")
