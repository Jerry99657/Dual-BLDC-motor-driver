"""Record firmware size and verify the official/core source boundary.

python tests/flash_report.py baseline
python tests/flash_report.py ui
python tests/flash_report.py final

Reports and baseline firmware/source copies live under ignored build/size-baseline.
Symbol sizes describe the linked LTO result, not exclusive source-file costs.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("label", nargs="?", default="current")
parser.add_argument("--directory", type=Path, default=Path("build/size-baseline"))
parser.add_argument("--allow-irq-user-code", action="store_true",
                    help="Permit only the R82 IRQ USER CODE blocks and the exact user-module CMake addition")
parser.add_argument("--allow-console-user-build",action="store_true",
                    help="Permit only the exact R88 link-section and four cold user-source Oz changes")
parser.add_argument("--allow-uart-user-code",action="store_true",
                    help="Permit only the exact R90 main/USART IRQ USER CODE routing and UART module addition")
parser.add_argument("--allow-spi-user-code",action="store_true",
                    help="Permit only R91 SPI/DMA USER CODE routing, user LCD transport calls and SPI module addition")
parser.add_argument("--allow-cubemx-user-code",action="store_true",
                    help="Permit only R92 IRQ USER CODE routing and the exact generated-config build check")
parser.add_argument("--allow-native-ll-init", action="store_true",
                    help="Permit R93 native CubeMX LL USART, its unchanged official sources, and SPI3 NSSP configuration")
parser.add_argument("--allow-diagnostics-user-code",action="store_true",
                    help="Permit only R95 CPU-vector/main USER CODE hooks and diagnostics module addition")
parser.add_argument("--allow-settings-user-build",action="store_true",
                    help="Permit only the exact R99 user settings service CMake source/flags addition")
parser.add_argument("--allow-key-startup-user-code",action="store_true",
                    help="Permit only the R103 main USER CODE cold-start KEY1 RC settling delay")
args = parser.parse_args()
OUT = ROOT / args.directory
LABEL = args.label


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def protected():
    paths = []
    for directory in ("Drivers", "Middlewares", "ThirdParty", "Core", "USB_Device",
                      "mycode/kk_ui", "mycode/kk_oled", "cmake"):
        paths.extend(p for p in (ROOT / directory).rglob("*") if p.is_file())
    paths += [ROOT / name for name in ("CMakeLists.txt", "CMakePresets.json", "STM32G431XX_FLASH.ld")]
    paths.extend(ROOT.glob("*.ioc"))
    return {p.relative_to(ROOT).as_posix(): sha(p) for p in paths}


def official_commands(mode):
    commands = json.loads((ROOT / "build" / mode / "compile_commands.json").read_text())
    result = {}
    for entry in commands:
        path = Path(entry["file"]).resolve().relative_to(ROOT).as_posix()
        if path.startswith(("Drivers/", "Middlewares/", "ThirdParty/", "Core/", "USB_Device/", "mycode/kk_")):
            result[path] = entry.get("command", entry.get("arguments"))
    return result


def verify_diagnostics_user_changes(changed):
    allowed={"Core/Src/main.c","Core/Src/stm32g4xx_it.c","CMakeLists.txt"}
    assert set(changed)<=allowed,changed
    for name in allowed:
        before=(OUT/name).read_text(encoding="utf-8")
        assert sha(OUT/name)==baseline["protected"][name],name
        expected=before
        if name.endswith("main.c"):
            expected=expected.replace('/* USER CODE BEGIN Includes */\n','/* USER CODE BEGIN Includes */\n#include "board_diag.h"\n')
            expected=expected.replace('/* USER CODE BEGIN 1 */\n','/* USER CODE BEGIN 1 */\n  BoardDiag_Init();\n')
            expected=expected.replace('  g_boot_stage = 4U;\n','  g_boot_stage = 4U;\n  if (BoardDiag_LastCrash()->kind) MotorSampling_GlobalStopIRQ();\n')
        elif name.endswith("stm32g4xx_it.c"):
            expected=expected.replace('/* USER CODE BEGIN Includes */\n','/* USER CODE BEGIN Includes */\n#include "board_diag.h"\n')
            for block,kind in [('NonMaskableInt',2),('HardFault',3),('MemoryManagement',4),('BusFault',5),('UsageFault',6)]:
                anchor='/* USER CODE BEGIN '+block+'_IRQn 0 */\n'
                assert expected.count(anchor)==1,block
                expected=expected.replace(anchor,anchor+'  BOARD_DIAG_VECTOR('+str(kind)+');\n')
        else:
            anchor='add_dependencies(${CMAKE_PROJECT_NAME} cubemx_config_check)\n'
            addition=('\n# User diagnostics retain only crash/first-fault records across MCU reset.\n'
                'target_sources(${CMAKE_PROJECT_NAME} PRIVATE mycode/board_diag.c)\n'
                'set_source_files_properties(mycode/board_diag.c PROPERTIES COMPILE_OPTIONS "-Oz;-flto")\n')
            assert expected.count(anchor)==1
            expected=expected.replace(anchor,anchor+addition)
        assert (ROOT/name).read_text(encoding="utf-8")==expected,"Unexpected protected change: "+name


def verify_irq_user_changes(changed):
    """Keep vendor files, generated initialization and compilation flags strict.

    This narrow exception permits board-owned vector routing, not edits to
    arbitrary Core blocks or any official driver implementation.
    """
    irq="Core/Src/stm32g4xx_it.c"
    assert set(changed)<= {irq,"CMakeLists.txt"},changed
    if irq in changed:
        blocks={"Includes","ADC1_2_IRQn 0","TIM1_BRK_TIM15_IRQn 0",
                "TIM2_IRQn 0","EXTI15_10_IRQn 0"}
        def outside_user(text):
            found=set()
            def replace(match):
                if match[2] not in blocks: return match[0]
                assert match[2] not in found,match[2]
                found.add(match[2])
                return match[1]+"\n<APPLICATION IRQ CODE>\n"+match[4]
            result=re.sub(r"(/\* USER CODE BEGIN ([^\n]*?) \*/)(.*?)(/\* USER CODE END \2 \*/)",
                          replace,text,flags=re.S)
            assert found==blocks,found
            return result
        assert outside_user((OUT/irq).read_text(encoding="utf-8"))==outside_user((ROOT/irq).read_text(encoding="utf-8")),irq
    if "CMakeLists.txt" in changed:
        addition=('\n# Application HAL/LL interrupt module; official driver options are unchanged.\n'
                  'target_sources(${CMAKE_PROJECT_NAME} PRIVATE mycode/motor_irq.c)\n'
                  'set_source_files_properties(mycode/motor_irq.c PROPERTIES COMPILE_OPTIONS "-Os;-flto")\n')
        current=(ROOT/"CMakeLists.txt").read_text(encoding="utf-8")
        assert current.count(addition)==1,"Unexpected build configuration change"
        assert current.replace(addition,"")==(OUT/"CMakeLists.txt").read_text(encoding="utf-8")


def verify_uart_user_changes(changed):
    allowed={"Core/Src/main.c","Core/Src/stm32g4xx_it.c","CMakeLists.txt"}
    assert set(changed)<=allowed,changed
    def replace_once(text,old,new):
        assert text.count(old)==1,old
        return text.replace(old,new)
    for name in allowed:
        before=(OUT/name).read_text(encoding="utf-8")
        assert sha(OUT/name)==baseline["protected"][name],"Invalid baseline copy: "+name
        expected=before
        if name=="Core/Src/main.c":
            expected=replace_once(expected,'#include "protocol.h"\n','#include "protocol.h"\n#include "uart_port.h"\n')
            expected=replace_once(expected,'static uint8_t rx_byte;\n','')
            callbacks=re.search(r'void HAL_UART_RxCpltCallback\(UART_HandleTypeDef \*huart\)\n\{.*?\n\}\n\nvoid HAL_UART_ErrorCallback\(UART_HandleTypeDef \*huart\)\n\{.*?\n\}\n\n',expected,re.S)
            assert callbacks and callbacks[0].count('HAL_UART_Receive_IT')==2
            expected=replace_once(expected,callbacks[0],'')
            expected=replace_once(expected,'  HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);','  Uart1_Init();')
        elif name=="Core/Src/stm32g4xx_it.c":
            expected=replace_once(expected,'#include "motor_irq.h"\n','#include "motor_irq.h"\n#include "uart_port.h"\n')
            expected=replace_once(expected,'  /* USER CODE BEGIN USART1_IRQn 0 */\n\n','  /* USER CODE BEGIN USART1_IRQn 0 */\n  Uart1_IRQHandler();\n  return;\n')
        else:
            anchor='set_source_files_properties(mycode/motor_irq.c PROPERTIES COMPILE_OPTIONS "-Os;-flto")\n'
            addition=('\n# USART1 runtime transport; generated HAL setup and ST sources stay intact.\n'
                'target_sources(${CMAKE_PROJECT_NAME} PRIVATE mycode/uart_port.c)\n'
                'set_source_files_properties(mycode/uart_port.c PROPERTIES COMPILE_OPTIONS "-Os;-flto")\n')
            expected=replace_once(expected,anchor,anchor+addition)
        assert (ROOT/name).read_text(encoding="utf-8")==expected,"Unexpected protected change: "+name


def verify_spi_user_changes(changed):
    allowed={"Core/Src/stm32g4xx_it.c","CMakeLists.txt","mycode/kk_oled/driver/kk_oled_driver.c"}
    assert set(changed)<=allowed,changed
    def once(text,old,new):
        assert text.count(old)==1,old
        return text.replace(old,new)
    for name in allowed:
        before=(OUT/name).read_text(encoding="utf-8")
        assert sha(OUT/name)==baseline["protected"][name],"Invalid baseline copy: "+name
        expected=before
        if name=="Core/Src/stm32g4xx_it.c":
            expected=once(expected,'#include "uart_port.h"\n','#include "uart_port.h"\n#include "spi_port.h"\n')
            for vector,call in {
                'DMA1_Channel1':'SpiPort_DMAIRQ(&hspi1,1U);',
                'DMA1_Channel2':'SpiPort_DMAIRQ(&hspi1,2U);',
                'DMA1_Channel4':'SpiPort_DMAIRQ(&hspi3,4U);',
                'DMA2_Channel1':'SpiPort_DMAIRQ(&hspi2,1U);',
                'DMA2_Channel2':'SpiPort_DMAIRQ(&hspi2,2U);',
                'SPI1':'SpiPort_IRQHandler(&hspi1);',
                'SPI2':'SpiPort_IRQHandler(&hspi2);',
                'SPI3':'SpiPort_IRQHandler(&hspi3);',
            }.items():
                block='  /* USER CODE BEGIN '+vector+'_IRQn 0 */\n'
                expected=once(expected,block+'\n',block+'  '+call+'\n  return;\n')
        elif name=="CMakeLists.txt":
            anchor='set_source_files_properties(mycode/uart_port.c PROPERTIES COMPILE_OPTIONS "-Os;-flto")\n'
            addition=('\n# SPI runtime and its five dedicated DMA channels; HAL still owns setup.\n'
                'target_sources(${CMAKE_PROJECT_NAME} PRIVATE mycode/spi_port.c)\n'
                'set_source_files_properties(mycode/spi_port.c PROPERTIES COMPILE_OPTIONS "-Os;-flto")\n')
            expected=once(expected,anchor,anchor+addition)
        else:
            expected=once(expected,'#include "spi.h"','#include "spi.h"\n#include "spi_port.h"')
            for old,new in [('HAL_SPI_Transmit_DMA','SpiPort_WriteDMA'),('HAL_SPI_Transmit','SpiPort_Write'),('HAL_SPI_Abort','SpiPort_Abort')]:
                expected=expected.replace(old,new)
            expected=once(expected,'void OLED_DriverService(void)\n{','void OLED_DriverService(void)\n{\n    SpiPort_Service(&hspi3);')
            expected=once(expected,' * SpiPort_Abort() 内部可能回调错误处理，因此这里允许重复调用。',
                          ' * 仅在主循环调用；底层复位 SPI3，丢弃尚未发送完的字节。')
        assert (ROOT/name).read_text(encoding="utf-8")==expected,"Unexpected protected change: "+name


def verify_cubemx_user_changes(changed):
    assert set(changed)<={"Core/Src/stm32g4xx_it.c","CMakeLists.txt"},changed
    name="Core/Src/stm32g4xx_it.c"
    before=(OUT/name).read_text(encoding="utf-8")
    expected=before.replace('#include "spi_port.h"','#include "spi_port.h"\n#include "cubemx_irq.h"')
    expected=re.sub(r'(  /\* USER CODE BEGIN DMA[12]_Channel\d_IRQn 0 \*/)\n  SpiPort_DMAIRQ\([^\n]+\n  return;\n',r'\1\n\n',expected)
    expected=expected.replace('  MotorIRQ_Key1();\n  return; /* Application owns this vector; keep generated fallback intact. */\n','\n')
    assert (ROOT/name).read_text(encoding="utf-8")==expected,name
    name="CMakeLists.txt"
    before=(OUT/name).read_text(encoding="utf-8")
    addition=('''# Every build checks the generated configuration against user-driver contracts.
# CubeMX remains the source of pin/clock/DMA initialization parameters.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_custom_target(cubemx_config_check
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_cubemx.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    VERBATIM)
add_dependencies(${CMAKE_PROJECT_NAME} cubemx_config_check)

''')
    anchor='# SPI runtime and its five dedicated DMA channels; HAL still owns setup.\n'
    assert before.count(anchor)==1
    assert (ROOT/name).read_text(encoding="utf-8")==before.replace(anchor,addition+anchor),name


def verify_native_ll_init(changed):
    verified = json.loads((ROOT/'build/cubemx-regeneration/verified.json').read_text())
    assert sha(ROOT/'FOC_two.ioc') == verified['ioc'], 'Run native CubeMX regeneration verification again'
    native = ROOT/verified['native_project']
    added = set(report['protected']) - set(baseline['protected'])
    ll_sources = {'Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_'+n+'.c'
                  for n in ('usart', 'gpio', 'rcc', 'utils', 'exti', 'dma')}
    assert added == ll_sources, added
    allowed = set(verified['sources']) | ll_sources | {
        'Core/Src/main.c', 'FOC_two.ioc', 'CMakeLists.txt', 'cmake/stm32cubemx/CMakeLists.txt'}
    assert set(changed) <= allowed, changed
    for name in added:
        assert sha(ROOT/name) == verified['official_sources'][name], name
    for name, expected in verified['sources'].items():
        assert sha(ROOT/name) == expected, 'Generated file differs from native CubeMX: '+name
    def read(name):
        path = OUT/('snapshot/'+name if name != 'CMakeLists.txt' else name)
        assert sha(path) == baseline['protected'][name], 'Invalid baseline '+name
        return path.read_text(encoding='utf-8')
    name = 'Core/Src/main.c'
    before = read(name)
    expected = re.sub(r'  /\* 0\. SPI2/SPI3[^\n]*\*/\n  CLEAR_BIT\(SPI2->CR2, SPI_CR2_NSSP\);\n  CLEAR_BIT\(SPI3->CR2, SPI_CR2_NSSP\);\n', '', before)
    assert before != expected and (ROOT/name).read_text(encoding='utf-8') == expected, name
    name = 'FOC_two.ioc'
    expected = read(name).replace('MX_USART1_UART_Init-USART1-false-HAL-true', 'MX_USART1_UART_Init-USART1-false-LL-true')
    expected = expected.replace('SPI3.IPParameters=VirtualType,Mode,Direction,BaudRatePrescaler,CalculateBaudRate,DataSize\n',
                                'SPI3.IPParameters=VirtualType,Mode,Direction,BaudRatePrescaler,CalculateBaudRate,DataSize,NSSPMode\n')
    expected = expected.replace('SPI3.Mode=SPI_MODE_MASTER\n', 'SPI3.Mode=SPI_MODE_MASTER\nSPI3.NSSPMode=SPI_NSS_PULSE_DISABLE\n')
    assert (ROOT/name).read_text(encoding='utf-8') == expected, name
    name = 'cmake/stm32cubemx/CMakeLists.txt'
    expected = read(name).replace('set(MX_Defines_Syms \n', 'set(MX_Defines_Syms \n\tUSE_FULL_LL_DRIVER \n')
    section = r'set\(STM32_Drivers_Src\n.*?\n\)'
    native_section = re.search(section, (native/name).read_text(encoding='utf-8'), re.S)[0]
    expected = re.sub(section, lambda m: native_section, expected, flags=re.S)
    assert (ROOT/name).read_text(encoding='utf-8') == expected, name
    name = 'CMakeLists.txt'
    expected = read(name).replace('# USART1 runtime transport; generated HAL setup and ST sources stay intact.',
                                 '# USART1 runtime transport; CubeMX owns HAL/LL initialization.')
    for n in ('uart', 'uart_ex'):
        expected = expected.replace('    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_'+n+'.c\n', '')
    anchor = 'set_property(SOURCE Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c\n'
    expected = expected.replace(anchor, anchor+''.join('    Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_'+n+'.c\n'
                                                       for n in ('usart', 'gpio', 'rcc')))
    assert (ROOT/name).read_text(encoding='utf-8') == expected, name
    # The only global compile change is CubeMX's native USE_FULL_LL_DRIVER.
    removed = {'Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_'+n+'.c' for n in ('uart', 'uart_ex')}
    for mode in ('Debug', 'Release'):
        old, now = baseline['commands'][mode], report['commands'][mode]
        assert set(old)-set(now) == removed, mode
        assert set(now)-set(old) == ll_sources, mode
        for path in set(old) & set(now):
            assert now[path].replace(' -DUSE_FULL_LL_DRIVER', '') == old[path], (mode, path)


def image(mode):
    folder = ROOT / "build" / mode
    elf, binary = folder / "FOC_two.elf", folder / "FOC_two.bin"
    symbols = subprocess.check_output(["arm-none-eabi-nm", "-S", "--size-sort", "--radix=d", str(elf)], text=True)
    functions = []
    for line in symbols.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[2] in "tT":
            functions.append({"name": parts[3], "bytes": int(parts[1])})
    sections = subprocess.check_output(["arm-none-eabi-objdump", "-h", str(elf)], text=True).splitlines()
    ram_end = 0x20000000
    for index, line in enumerate(sections[:-1]):
        parts = line.split()
        if len(parts) >= 7 and parts[0].isdigit() and "ALLOC" in sections[index + 1]:
            size, address = int(parts[2], 16), int(parts[3], 16)
            if 0x20000000 <= address < 0x20008000:
                ram_end = max(ram_end,address+size)
    return {"flash_bytes": binary.stat().st_size, "flash_free": 131072 - binary.stat().st_size,
            "ram_allocated": ram_end-0x20000000, "bin_sha256": sha(binary), "elf_sha256": sha(elf),
            "largest_functions": list(reversed(functions[-50:]))}


OUT.mkdir(parents=True, exist_ok=True)
report = {"protected": protected(), "commands": {}, "images": {}}
for mode in ("Debug", "Release"):
    report["commands"][mode] = official_commands(mode)
    report["images"][mode] = image(mode)
if LABEL == "baseline":
    if (OUT / "baseline.json").exists():
        raise SystemExit("Baseline already exists; do not overwrite the original reference.")
    for mode in ("Debug", "Release"):
        destination = OUT / mode
        destination.mkdir(exist_ok=True)
        for suffix in ("elf", "bin", "map"):
            shutil.copy2(ROOT / "build" / mode / ("FOC_two." + suffix), destination)
    for path in (ROOT / "mycode").glob("*.[ch]"):
        destination = OUT / "mycode" / path.name
        destination.parent.mkdir(exist_ok=True)
        shutil.copy2(path, destination)
    for name in ("Core/Src/main.c","Core/Src/stm32g4xx_it.c","CMakeLists.txt","mycode/kk_oled/driver/kk_oled_driver.c"):
        destination=OUT/name
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/name,destination)
else:
    baseline = json.loads((OUT / "baseline.json").read_text())
    if baseline["protected"] != report["protected"]:
        changed = [name for name in set(baseline["protected"]) | set(report["protected"])
                   if baseline["protected"].get(name) != report["protected"].get(name)]
        if args.allow_key_startup_user_code:
            assert changed==["Core/Src/main.c"],changed
            name="Core/Src/main.c"
            assert sha(OUT/name)==baseline["protected"][name],"Invalid baseline main.c"
            before=(OUT/name).read_text(encoding="utf-8")
            anchor='  /* Also latch a key held before power-up (there may be no falling edge). */\n'
            addition=('  /* KEY1 uses the generated internal pull-up and a 100 nF capacitor.\n'
                      '   * Allow cold-start RC charging before polling its level (25..55 kohm).\n'
                      '   * PWM/ADC have not started; EXTI stays live and no stop latch is cleared. */\n'
                      '  HAL_Delay(30U);\n')
            assert before.count(anchor)==1
            expected=before.replace(anchor,addition+anchor)
            assert (ROOT/name).read_text(encoding="utf-8")==expected,"Unexpected main.c change"
            report["permitted_user_changes"]=changed
        elif args.allow_settings_user_build:
            assert changed==["CMakeLists.txt"],changed
            addition=('\n# Settings business is independent of the LCD/application presentation.\n'
                      'target_sources(${CMAKE_PROJECT_NAME} PRIVATE mycode/app_settings.c)\n'
                      'set_source_files_properties(mycode/app_settings.c PROPERTIES COMPILE_OPTIONS "-Oz;-flto")\n')
            current=(ROOT/"CMakeLists.txt").read_text(encoding="utf-8")
            assert current.count(addition)==1
            assert current.replace(addition,"")==(OUT/"CMakeLists.txt").read_text(encoding="utf-8"),"Unexpected build configuration change"
            report["permitted_user_changes"]=changed
        elif args.allow_diagnostics_user_code:
            verify_diagnostics_user_changes(changed)
            report["permitted_user_changes"]=sorted(changed)
        elif args.allow_native_ll_init:
            verify_native_ll_init(changed)
            report['permitted_native_init_changes'] = sorted(changed)
        elif args.allow_console_user_build:
            assert changed==["CMakeLists.txt"],changed
            before=(OUT/"CMakeLists.txt").read_text(encoding="utf-8")
            expected=before.replace("-fno-function-sections -fno-data-sections -flto=1",
                                    "-ffunction-sections -fdata-sections -flto=1")
            expected=expected.replace("-flto-partition=one)","-flto-partition=one -Wl,--sort-section=alignment)")
            expected=expected.replace("# Co-locate LTO-generated functions/data to share literal pools and padding.",
                                      "# Discard unused LTO sections and reduce padding by sorting their alignment.")
            anchor='set_source_files_properties(mycode/ui_app.c PROPERTIES COMPILE_OPTIONS "-Oz;-flto;-fno-inline-functions-called-once")\n'
            assert expected.count(anchor)==1
            expected=expected.replace(anchor,anchor+'set_source_files_properties(mycode/app_storage.c mycode/w25q64.c mycode/ws2812.c mycode/number_parse.c\n    PROPERTIES COMPILE_OPTIONS "-Oz;-flto")\n')
            assert (ROOT/"CMakeLists.txt").read_text(encoding="utf-8")==expected,"Unexpected build configuration change"
            report["permitted_user_changes"]=changed
        elif args.allow_cubemx_user_code:
            verify_cubemx_user_changes(changed)
            report["permitted_user_changes"]=sorted(changed)
        elif args.allow_spi_user_code:
            verify_spi_user_changes(changed)
            report["permitted_user_changes"]=sorted(changed)
        elif args.allow_uart_user_code:
            verify_uart_user_changes(changed)
            report["permitted_user_changes"]=sorted(changed)
        elif args.allow_irq_user_code:
            verify_irq_user_changes(changed)
            report["permitted_user_changes"]=sorted(changed)
        else:
            raise SystemExit("Protected files changed: " + ", ".join(sorted(changed)))
    if not args.allow_native_ll_init and baseline["commands"] != report["commands"]:
        raise SystemExit("Official/core compilation commands changed.")
    for mode, current in report["images"].items():
        current["saved_bytes"] = baseline["images"][mode]["flash_bytes"] - current["flash_bytes"]
(OUT / (LABEL + ".json")).write_text(json.dumps(report, indent=2), encoding="utf-8")
for mode, current in report["images"].items():
    print(mode, json.dumps({k: v for k, v in current.items() if k != "largest_functions"}))
print("Protected files:", len(report["protected"]),
      "; official/core compile commands", "recorded." if LABEL == "baseline" else "verified.")
if "permitted_user_changes" in report:
    print("Verified application-only exceptions:",", ".join(report["permitted_user_changes"]))
