# @file run_probe_spi.py
# @author 煲仔卤煮(ZealerLu)
# @date 2026-09-07
#
# @copyright Copyright (c) 2026
#
"""Compile production SPI state machine against hardware function replacements.
Uses the real AT32 types/constants and Cortex-M instruction emulation.
"""
from pathlib import Path
import subprocess
import sys
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

root = Path(__file__).resolve().parents[1]
build = root / "tests/build"
build.mkdir(exist_ok=True)
compiler = Path("C:/Keil_v5/ARM/ARMCC/Bin")
component = ("irq" if "--irq" in sys.argv else
             "board_check" if "--board_check" in sys.argv else
             "calib" if "--calib" in sys.argv else
             "attitude" if "--attitude" in sys.argv else
             "led" if "--led" in sys.argv else
             "frame" if "--frame" in sys.argv else
             "usb" if "--usb" in sys.argv else
             "proto" if "--proto" in sys.argv else
             "fifo" if "--fifo" in sys.argv else
             ("validation" if "--validation" in sys.argv else "spi"))
includes = ["tests/stubs/validation", "project/inc", "middlewares/sensors", "middlewares/sensors/adxl357", "middlewares/sensors/icm42688", "libraries/drivers/inc", "libraries/cmsis/cm4/device_support", "libraries/cmsis/cm4/core_support"]
objects = []
package = root / "middlewares/agm"
link_extra = []
if "--attitude" in sys.argv:
    link_extra = [str(package / "lib/libInvnAlgoAGM.a")]
project_sources = {
    "irq": ["at32f413_int"],
    "board_check": ["probe_board_check", "probe_calib", "probe_frame"],
    "spi": ["probe_spi"],
    "frame": ["probe_frame"],
    "led": ["probe_led"],
    "attitude": ["probe_attitude"],
    "calib": ["probe_calib", "probe_frame"],
    "usb": ["probe_usb"],
    "proto": ["probe_proto", "probe_frame", "probe_acquisition"],
    "validation": ["probe_validation"],
}
sources = [root / f"tests/test_probe_{component}.c"]
if component == "fifo":
    sources = [root / "tests/test_sensor_fifo.c"]
for name in project_sources.get(component, []):
    sources.append(root / f"project/src/{name}.c")
if component in ("validation", "fifo", "proto", "attitude", "calib", "board_check"):
    sources += [root / f"middlewares/sensors/{name}/{name}.c" for name in ("adxl357", "icm42688")]
if component == "attitude":
    includes.append(str(package / "include"))
extra_flags = ["--wchar32"] if component == "attitude" else []
if component == "usb":
    includes += ["middlewares/usbd_drivers/inc", "middlewares/usbd_class/cdc"]
if component == "irq":
    extra_flags += ["--split_sections", "-DPROBE_HARDWARE_VALIDATION=" + sys.argv[sys.argv.index("--irq") + 1]]
for source in sources:
    obj = build / (source.stem + ".o")
    subprocess.run([str(compiler / "armcc.exe"), "--cpu=Cortex-M4", "--c99", "-O1", *extra_flags,
                    "-DAT32F413CCU7", "-DUSE_STDPERIPH_DRIVER",
                    *["-I" + str(root / p) for p in includes],
                    "-c", str(source), "-o", str(obj)], check=True)
    objects.append(str(obj))
target = build / f"{component}-test.axf"
subprocess.run([str(compiler / "armlink.exe"), *objects, *link_extra, "--entry=test_main",
                "--ro-base=0x10000", "--rw-base=0x20000000", "-o", str(target)], check=True)
machine = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
machine.mem_map(0x10000, 0x10000)
machine.mem_map(0x20000000, 0x20000)
machine.mem_map(0x40010000, 0x10000)  # APB2 peripherals (USART1 sts/dt reads)
if component in ("calib", "board_check"):
    machine.mem_map(0x08000000, 0x40000)  # flash alias for calibration records
with target.open("rb") as stream:
    elf = ELFFile(stream)
    for section in elf.iter_sections():
        if section["sh_flags"] & 2 and section["sh_type"] != "SHT_NOBITS":
            machine.mem_write(section["sh_addr"], section.data())
    entry = elf.get_section_by_name(".symtab").get_symbol_by_name("test_main")[0]["st_value"]
machine.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
machine.reg_write(UC_ARM_REG_LR, 0x1F001)
machine.emu_start(entry | 1, 0x1F000, count=2000000000)
if machine.reg_read(UC_ARM_REG_PC) != 0x1F000:
    raise SystemExit("FAIL: test did not return")
result = machine.reg_read(UC_ARM_REG_R0)
if result:
    raise SystemExit(f"FAIL: test_probe_{component}.c line {result}")
messages = {
    "irq": "EXINT dispatch to the component selected by build mode",
    "board_check": "RTT restart ownership, six-face capture, save guards and LED override",
    "spi": "DMA ownership, independent buses, busy tail, errors and retry",
    "validation": "M2 register sequence, identity, signed decoding and retry",
    "fifo": "configuration ranges, FIFO headers, markers, invalid data and bounds",
    "frame": "CRC-32C vectors, byte-exact builds, resync, type caps and continuity",
    "led": "state colours, blink phases, polarity and dark states",
    "attitude": "passband gain, stopband rejection, static AGM convergence",
    "calib": "six-position fit, gates, dual-record flash and fallback",
    "usb": "packet chunking, busy semantics, RX ring, overflow and timeout abort",
    "proto": "session, idempotency, config validation, fetch window and fault injection",
}
print("PASS: " + messages[component])
