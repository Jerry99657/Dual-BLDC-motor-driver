"""Check the linked STM32 images, not just the spelling of HAL_Init in C.

Run after building both presets. Detects a call swallowed by a C comment and
checks the actual strong MSP override survived linking/optimization.
"""
import argparse
import hashlib
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

repo = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--toolchain-bin", type=Path)
args = parser.parse_args()

if args.toolchain_bin:
    tool_dir = args.toolchain_bin
elif shutil.which("arm-none-eabi-objdump"):
    tool_dir = Path(shutil.which("arm-none-eabi-objdump")).parent
else:
    cache = (repo / "build/Debug/CMakeCache.txt").read_text(encoding="utf-8")
    compiler = re.search(r"^CMAKE_C_COMPILER:[^=]+=(.+)$", cache, re.M)
    if not compiler:
        parser.error("build Debug first or pass --toolchain-bin")
    tool_dir = Path(compiler[1].strip()).parent


def tool(name, *arguments):
    executable = tool_dir / ("arm-none-eabi-" + name)
    if not executable.exists():
        executable = executable.with_suffix(".exe")
    return subprocess.check_output([str(executable), *map(str, arguments)], text=True)


def calls(disassembly):
    return re.findall(r"\bblx?(?:\.[nw])?\s+[0-9a-f]+\s+<([^>]+)>", disassembly)


for mode in ("Debug", "Release"):
    root = repo / "build" / mode
    elf = root / "FOC_two.elf"
    symbols = tool("nm", elf)
    for symbol in ("HAL_Init", "HAL_MspInit"):
        assert re.search(r"^\w+ T " + symbol + r"$", symbols, re.M), (mode, symbol)
    for symbol in ("g_boot_stage", "g_error_return_address"):
        assert re.search(r"^\w+ [bB] " + symbol + r"$", symbols, re.M), (mode, symbol)

    main_calls = calls(tool("objdump", "-d", "--disassemble=main", elf))
    hardware_calls = [c for c in main_calls if c.startswith(("HAL_", "MX_", "SystemClock_Config"))]
    assert hardware_calls[0] == "HAL_Init", (mode, hardware_calls[:5])
    assert any(c.startswith(("SystemClock_Config", "HAL_RCC_")) for c in hardware_calls[1:])
    hal_calls = calls(tool("objdump", "-d", "--disassemble=HAL_Init", elf))
    assert hal_calls.index("HAL_InitTick") < hal_calls.index("HAL_MspInit"), (mode, hal_calls)

    image = (root / "FOC_two.bin").read_bytes()
    assert b"UI-R36-SMOOTH" in image and b"UI-R35-TRACK" not in image, mode
    with tempfile.TemporaryDirectory(prefix="startup-", dir=repo / "build") as scratch:
        output = Path(scratch) / "verified.bin"
        tool("objcopy", "-O", "binary", elf, output)
        assert image == output.read_bytes(), mode
    print(f"PASS: {mode}: HAL_Init precedes clocks/peripherals; strong MSP; R36; "
          f"ELF/BIN match ({len(image)} B), SHA256 {hashlib.sha256(image).hexdigest()}")
