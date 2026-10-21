from pathlib import Path
import shutil
import subprocess

repo = Path(__file__).resolve().parents[2]
output = repo / "build/ui-validation"
output.mkdir(parents=True, exist_ok=True)
sources = list((repo / "mycode/kk_ui/src").glob("*.c"))
sources += list((repo / "mycode/kk_oled/graphics").glob("*.c"))
sources += [repo / p for p in ("mycode/ui_app.c", "mycode/ui_input.c", "mycode/ui_fonts.c", "mycode/motor_params.c", "tests/ui_host/test_ui.c", "tests/motor_host/control_stub.c")]
includes = ["tests/ui_host", "mycode", "mycode/kk_ui/include", "mycode/kk_ui/src", "mycode/kk_oled/include", "mycode/kk_oled/graphics", "mycode/kk_oled/driver"]
command = [shutil.which("gcc") or "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1", "-DKK_OLED_TEST", "-DCONTROL_STUB_UI"]
command += ["-I" + str(repo / p) for p in includes]
command += [str(p) for p in sources] + ["-lm", "-o", str(output / "test_ui.exe")]
subprocess.run(command, cwd=repo, check=True)
subprocess.run([str(output / "test_ui.exe")], cwd=repo, check=True)

# Exercise the real storage state machine against an asynchronous NOR model.
subprocess.run([shutil.which("gcc") or "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1", "-Itests/ui_host", "-Imycode", "mycode/app_storage.c", "tests/ui_host/test_storage.c", "-o", str(output / "test_storage.exe")], cwd=repo, check=True)
subprocess.run([str(output / "test_storage.exe")], cwd=repo, check=True)

# Run the actual ST7735 driver; model an interrupt before DMA-start returns.
command = [shutil.which("gcc") or "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1", "-DKK_OLED_TEST", "-Itests/oled_driver_host", "-Imycode/kk_oled/include", "-Imycode/kk_oled/graphics", "-Imycode/kk_oled/driver"]
command += [str(p) for p in (repo / "mycode/kk_oled/graphics").glob("*.c")]
command += ["mycode/kk_oled/driver/kk_oled_driver.c", "tests/oled_driver_host/test_driver.c", "-lm", "-o", str(output / "test_driver.exe")]
subprocess.run(command, cwd=repo, check=True)
subprocess.run([str(output / "test_driver.exe")], cwd=repo, check=True)
