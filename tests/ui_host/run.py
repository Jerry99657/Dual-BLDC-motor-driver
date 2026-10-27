from pathlib import Path
import argparse
import shutil
import subprocess
import sys

parser=argparse.ArgumentParser(description="UI/storage/display regressions and optional diagnostic-query baseline comparison")
parser.add_argument("--query-baseline",type=Path,help="Previous ui_app.c to compile against the same deterministic host fixture")
args=parser.parse_args()
repo = Path(__file__).resolve().parents[2]
output = repo / "build/ui-validation"
output.mkdir(parents=True, exist_ok=True)
subprocess.run([sys.executable, "scripts/pack_ui_resources.py", "--check"], cwd=repo, check=True)
subprocess.run([shutil.which("gcc") or "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1", "-Imycode", "mycode/ui_text.c", "tests/ui_host/test_ui_text.c", "-o", str(output / "test_ui_text.exe")], cwd=repo, check=True)
subprocess.run([str(output / "test_ui_text.exe")], cwd=repo, check=True)
sources = list((repo / "mycode/kk_ui/src").glob("*.c"))
sources += list((repo / "mycode/kk_oled/graphics").glob("*.c"))
sources += [repo / p for p in ("mycode/ui_app.c", "mycode/ui_text.c", "mycode/ui_input.c", "mycode/ui_fonts.c", "mycode/motor_params.c", "tests/ui_host/test_ui.c", "tests/motor_host/control_stub.c")]
sources += [repo / "mycode/motor_math.c",repo / "mycode/app_settings.c"]
sources += [repo / "tests/diag_stub.c"]
includes = ["tests/ui_host", "mycode", "mycode/kk_ui/include", "mycode/kk_ui/src", "mycode/kk_oled/include", "mycode/kk_oled/graphics", "mycode/kk_oled/driver"]
command = [shutil.which("gcc") or "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1", "-DKK_OLED_TEST", "-DCONTROL_STUB_UI"]
command += ["-I" + str(repo / p) for p in includes]
command += [str(p) for p in sources] + ["-Wl,--wrap=OLED_DrawUTF8", "-lm", "-o", str(output / "test_ui.exe")]
subprocess.run(command, cwd=repo, check=True)
subprocess.run([str(output / "test_ui.exe")], cwd=repo, check=True)
subprocess.run([str(output / "test_ui.exe"),"headless"], cwd=repo, check=True)
subprocess.run([str(output / "test_ui.exe"),"queries"], cwd=repo, check=True)
subprocess.run([str(output / "test_ui.exe"),"align"], cwd=repo, check=True)
subprocess.run([str(output / "test_ui.exe"),"settings"], cwd=repo, check=True)
metrics=subprocess.check_output([str(output / "test_ui.exe"),"metrics"],cwd=repo,text=True)
(output/"query-metrics-current.txt").write_text(metrics)
if args.query_baseline:
    baseline_command=command.copy()
    baseline_path=args.query_baseline.resolve()
    # Old UI owned App_SaveState. Link its row formatter alongside the new
    # service without two public definitions; metrics do not exercise saving.
    legacy=baseline_path.read_text(encoding='utf-8')
    legacy=legacy.replace('unsigned App_SaveState(void)','unsigned Baseline_SaveState(void)')
    resources=(baseline_path.parent/'ui_resources_packed.h').as_posix()
    legacy=legacy.replace('#include "ui_resources_packed.h"',f'#include "{resources}"')
    baseline_source=output/'query-baseline-ui.c';baseline_source.write_text(legacy,encoding='utf-8')
    baseline_command[baseline_command.index(str(repo/"mycode/ui_app.c"))]=str(baseline_source)
    baseline_command[-1]=str(output/"test_ui_baseline.exe")
    subprocess.run(baseline_command,cwd=repo,check=True)
    previous=subprocess.check_output([baseline_command[-1],"metrics"],cwd=repo,text=True)
    (output/"query-metrics-baseline.txt").write_text(previous)
    for before,after in zip(previous.splitlines(),metrics.splitlines()):
        assert before.split(":")[0]==after.split(":")[0]
        counts=lambda line:sum(int(item.split("=")[1]) for item in line.split(":")[1].split())
        assert counts(after)<counts(before),(before,after)
        print(f'{after.split(":")[0]} snapshot calls: {counts(before)} -> {counts(after)}')

# Exercise the real storage state machine against an asynchronous NOR model.
subprocess.run([shutil.which("gcc") or "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1", "-Itests/ui_host", "-Imycode", "mycode/app_storage.c", "tests/ui_host/test_storage.c", "-o", str(output / "test_storage.exe")], cwd=repo, check=True)
subprocess.run([str(output / "test_storage.exe")], cwd=repo, check=True)

# Run the actual ST7735 driver; model an interrupt before DMA-start returns.
command = [shutil.which("gcc") or "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1", "-DKK_OLED_TEST", "-Itests/oled_driver_host", "-Imycode", "-Imycode/kk_oled/include", "-Imycode/kk_oled/graphics", "-Imycode/kk_oled/driver"]
command += [str(p) for p in (repo / "mycode/kk_oled/graphics").glob("*.c")]
command += ["mycode/kk_oled/driver/kk_oled_driver.c", "tests/oled_driver_host/test_driver.c", "-lm", "-o", str(output / "test_driver.exe")]
subprocess.run(command, cwd=repo, check=True)
subprocess.run([str(output / "test_driver.exe")], cwd=repo, check=True)
