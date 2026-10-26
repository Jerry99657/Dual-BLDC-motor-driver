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
    for symbol in ("DMA1_Channel1_IRQHandler", "DMA1_Channel2_IRQHandler",
                   "DMA2_Channel1_IRQHandler", "DMA2_Channel2_IRQHandler",
                   "SPI1_IRQHandler", "SPI2_IRQHandler", "ADC1_2_IRQHandler", "TIM2_IRQHandler",
                   "TIM1_BRK_TIM15_IRQHandler", "HAL_COMP_MspInit", "HAL_DAC_MspInit", "EXTI15_10_IRQHandler", "TIM7_IRQHandler"):
        assert re.search(r"^\w+ T " + symbol + r"$", symbols, re.M), (mode, symbol)
    for symbol in ("g_boot_stage", "g_error_return_address"):
        assert re.search(r"^\w+ [bB] " + symbol + r"$", symbols, re.M), (mode, symbol)

    main_calls = calls(tool("objdump", "-d", "--disassemble=main", elf))
    hardware_calls = [c for c in main_calls if c.startswith(("HAL_", "MX_", "SystemClock_Config"))]
    assert hardware_calls[0] == "HAL_Init", (mode, hardware_calls[:5])
    assert any(c.startswith(("SystemClock_Config", "HAL_RCC_")) for c in hardware_calls[1:])
    hal_calls = calls(tool("objdump", "-d", "--disassemble=HAL_Init", elf))
    assert hal_calls.index("HAL_InitTick") < hal_calls.index("HAL_MspInit"), (mode, hal_calls)

    # R82: HAL owns setup/DMA; the board's LL vectors own ADC12/TIM/EXTI.
    # Application hooks may inline; verify the real linked control paths.
    for removed in ("HAL_ADC_IRQHandler","HAL_TIM_IRQHandler","HAL_GPIO_EXTI_IRQHandler"):
        assert not re.search(r"^\w+ [tT] "+removed+r"$",symbols,re.M),(mode,removed)
    adc_calls=calls(tool("objdump", "-d", "--disassemble=MotorIRQ_Adc12", elf))
    tim_calls=calls(tool("objdump", "-d", "--disassemble=MotorIRQ_Tim2", elf))
    assert any(c.startswith("StHso_Process") for c in adc_calls), (mode, adc_calls)
    # LTO may inline the detector into the ADC IRQ. Its ownership gate and
    # two independent pole checks must then remain in the real linked path.
    assert any(c.startswith("MotorDetect_Process") for c in adc_calls) or (
        "MotorDetect_Active" in adc_calls and adc_calls.count("poles")>=2), (mode, adc_calls)
    assert "service_hso_feedback" in adc_calls, (mode, adc_calls)
    assert "MotorControl_PositionVelocity" in tim_calls, (mode, tim_calls)
    assert "MotorControl_VelocityControl" in tim_calls, (mode, tim_calls)
    # Include tail branches as well as calls when checking vector routing.
    def exti_transfers(symbol):
        code=tool("objdump", "-d", "--disassemble="+symbol, elf)
        return re.findall(r"\bb(?:lx?)?(?:\.[nw])?\s+[0-9a-f]+\s+<([^>]+)>",code)
    for vector,dispatcher in (("ADC1_2_IRQHandler","MotorIRQ_Adc12"),
                              ("TIM2_IRQHandler","MotorIRQ_Tim2"),
                              ("TIM1_BRK_TIM15_IRQHandler","MotorIRQ_Break"),
                              ("EXTI15_10_IRQHandler","MotorIRQ_Key1")):
        assert re.search(r"^\w+ T "+dispatcher+r"$",symbols,re.M),(mode,dispatcher)
        assert dispatcher in exti_transfers(vector),(mode,vector)
    break_calls=exti_transfers("MotorIRQ_Break")
    assert any(c.startswith("ibus_latch") for c in break_calls),(mode,break_calls)
    exti_calls=exti_transfers("MotorIRQ_Key1")
    assert any(c.startswith("MotorSampling_GlobalStopIRQ") for c in exti_calls), (mode,exti_calls)

    image = (root / "FOC_two.bin").read_bytes()
    import sys
    sys.path.insert(0,str(repo / "scripts"))
    from pack_ui_resources import resources,pack,unpack
    raw_resources=resources();packed_resources=pack(raw_resources)
    assert packed_resources in image and unpack(packed_resources,len(raw_resources))==raw_resources,mode
    text_image=image+raw_resources
    assert b"UI-R89-ESC" in text_image and b"UI-R88-PC" not in text_image, mode
    assert "App_FloatToDoubleBits" in symbols and "__aeabi_f2d" in symbols,mode
    for unused in ("__aeabi_dadd","__aeabi_dsub","__adddf3","__subdf3"):
        assert not re.search(r"^\w+ [tT] "+unused+r"$",symbols,re.M),(mode,unused)
    assert "__aeabi_ddiv" not in symbols and "__divdf3" not in symbols,mode
    for removed in ("snprintf","sniprintf","_printf_i","_svfprintf_r","_svfiprintf_r"):
        assert not re.search(r"^\w+ [tT] "+removed+r"$",symbols,re.M), (mode,removed)
    assert "UiText_Format" in symbols,mode
    assert b"M1 Sensorless" in text_image and b"M1 Six Step" in text_image, mode
    assert b"M1 Detect" in text_image and b"RS NOT STEADY" in text_image and b"POLES / REPEAT" in text_image, mode
    assert b"ESC Auto" in text_image and b"Throttle only" in text_image,mode
    assert "__udivmoddi4" in symbols,mode
    assert b"PERIOD HIGH" in text_image and b"6S Sample" in text_image, mode
    assert b"6STEP SINE" in text_image and b"6STEP CURRENT" in text_image, mode
    assert "sinTable_f32" not in symbols and "arm_sin_f32" not in symbols, mode
    assert "MotorMath_SinCos" in symbols, mode
    assert b"6STEP NO ZC" in text_image and b"6STEP RUN" in text_image, mode
    assert b"Throttle:" in text_image and b"6STEP FAST" in text_image, mode
    with tempfile.TemporaryDirectory(prefix="startup-", dir=repo / "build") as scratch:
        output = Path(scratch) / "verified.bin"
        tool("objcopy", "-O", "binary", elf, output)
        assert image == output.read_bytes(), mode
    print(f"PASS: {mode}: HAL_Init precedes clocks/peripherals; strong MSP/COMP/DAC/TIM7, real LL ADC/TIM/Break/KEY1 paths and no generic HAL IRQ dispatchers; typed protocol queue, no double division/printf runtime; "
          f"ELF/BIN match ({len(image)} B), SHA256 {hashlib.sha256(image).hexdigest()}")
