from pathlib import Path
import shutil
import subprocess

repo = Path(__file__).resolve().parents[2]
out = repo / "build/usb-validation"
out.mkdir(parents=True, exist_ok=True)
includes = ["tests/usb_host", "USB_Device/App", "mycode", "tests/motor_host",
            "Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc",
            "Middlewares/ST/STM32_USB_Device_Library/Core/Inc"]
command = [shutil.which("gcc") or "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1"]
command += ["-I" + p for p in includes]
command += ["USB_Device/App/usbd_cdc_if.c", "tests/usb_host/test_cdc_if.c",
            "-o", str(out / "test_cdc_if.exe")]
subprocess.run(command, cwd=repo, check=True)
subprocess.run([str(out / "test_cdc_if.exe")], cwd=repo, check=True)
