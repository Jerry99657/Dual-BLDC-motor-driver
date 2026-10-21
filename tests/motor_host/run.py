from pathlib import Path
import shutil
import subprocess

repo = Path(__file__).resolve().parents[2]
out = repo / "build/motor-validation"
out.mkdir(parents=True, exist_ok=True)
compiler = shutil.which("gcc") or "gcc"
for name, sources in {
    "control": ["mycode/JerryFOC.c", "mycode/motor_control.c", "mycode/protocol.c", "mycode/motor_params.c", "tests/motor_host/test_control.c"],
    "profiles": ["mycode/motor_params.c", "tests/motor_host/test_profiles.c"],
    "sampling": ["mycode/motor_sampling.c", "mycode/motor_params.c", "tests/motor_host/test_sampling.c"],
    "encoder": ["mycode/MT6701.c", "mycode/w25q64.c", "mycode/app_storage.c", "tests/motor_host/test_encoder.c"],
    "foc": ["mycode/JerryFOC.c", "mycode/protocol.c", "mycode/motor_params.c", "tests/motor_host/test_foc.c"],
    "hso": ["mycode/st_hso.c", "mycode/motor_params.c", "tests/motor_host/test_hso.c"],
    "protocol": ["mycode/protocol.c", "mycode/motor_params.c", "tests/motor_host/test_protocol.c"],
}.items():
    if name in ("foc","protocol"):
        sources += ["tests/motor_host/control_stub.c"]
    command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1",
               "-Itests/motor_host", "-Imycode", "-IThirdParty/ST_HSO", *sources, "-lm", "-o", str(out / (name + ".exe"))]
    subprocess.run(command, cwd=repo, check=True)
    subprocess.run([str(out / (name + ".exe"))], cwd=repo, check=True)
    if name == "encoder":
        for scenario in ("missing", "timeout", "missing-wrap"):
            subprocess.run([str(out / "encoder.exe"), scenario], cwd=repo, check=True)
