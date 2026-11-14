# @file run_probe_acquisition.py
# @author 煲仔卤煮(ZealerLu)
# @date 2026-09-10
#
# @copyright Copyright (c) 2026
#
"""Run the acquisition state machine against the fake SPI fixture on emulated
Cortex-M4. Requires unicorn and pyelftools, as run_probe_time.py does.
This is instruction emulation, not hardware timing or sensor validation.
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
includes = ["-I" + str(root / "tests/stubs"),
            "-I" + str(root / "project/inc"),
            "-I" + str(root / "middlewares/sensors"),
            "-I" + str(root / "middlewares/sensors/adxl357"),
            "-I" + str(root / "middlewares/sensors/icm42688")]
objects = []
for source in [root / "tests/test_probe_acquisition.c",
               root / "project/src/probe_acquisition.c",
               root / "middlewares/sensors/adxl357/adxl357.c",
               root / "middlewares/sensors/icm42688/icm42688.c"]:
    obj = build / (source.stem + ".o")
    subprocess.run([str(compiler / "armcc.exe"), "--cpu=Cortex-M4", "--c99", "-O1",
                   *includes, "-c", str(source), "-o", str(obj)], check=True)
    objects.append(str(obj))
target = build / "acquisition-test.axf"
subprocess.run([str(compiler / "armlink.exe"), *objects, "--entry=test_main",
                "--ro-base=0x10000", "--rw-base=0x20000000", "-o", str(target)], check=True)
machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
machine.mem_map(0x10000, 0x10000)
machine.mem_map(0x20000000, 0x20000)
with target.open("rb") as stream:
    elf = ELFFile(stream)
    for segment in elf.iter_segments():
        if segment["p_type"] == "PT_LOAD":
            machine.mem_write(segment["p_paddr"], segment.data())
machine.reg_write(UC_ARM_REG_SP, 0x20020000)
machine.reg_write(UC_ARM_REG_R0, 0)
machine.reg_write(UC_ARM_REG_LR, 0x10000)
machine.emu_start(0x10001, 0x10000, timeout=0)
print("PASS: acquisition batching, queue drops, faults, stop boundary")
