from pathlib import Path
import shutil
import subprocess
import sys

repo = Path(__file__).resolve().parents[2]
out = repo / "build/motor-validation"
out.mkdir(parents=True, exist_ok=True)
(repo / "build/console-r87").mkdir(parents=True, exist_ok=True)
(repo / "build/console-r88").mkdir(parents=True, exist_ok=True)
(repo / "build/telemetry-r86").mkdir(parents=True, exist_ok=True)
compiler = shutil.which("gcc") or "gcc"
subprocess.run([sys.executable, "tests/motor_host/check_spi_clock.py"],
               cwd=repo, check=True)
suites = {
    "inverter": ["mycode/JerryFOC.c", "mycode/protocol.c", "mycode/number_parse.c", "mycode/motor_params.c", "tests/motor_host/test_inverter.c"],
    "current": ["mycode/JerryFOC.c", "mycode/protocol.c", "mycode/number_parse.c", "mycode/motor_params.c", "tests/motor_host/test_current.c"],
    "estimator": ["mycode/encoder_estimator.c","tests/motor_host/test_estimator.c"],
    "angle": ["mycode/angle_src_encoder.c","mycode/motor_math.c","tests/motor_host/test_angle.c"],
    "detect": ["mycode/JerryFOC.c", "mycode/motor_control.c", "mycode/protocol.c", "mycode/number_parse.c", "mycode/motor_params.c", "tests/motor_host/test_detect.c"],
    "control": ["mycode/JerryFOC.c", "mycode/motor_control.c", "mycode/protocol.c", "mycode/number_parse.c", "mycode/motor_params.c", "tests/motor_host/test_control.c"],
    "motion": ["mycode/JerryFOC.c", "mycode/motor_control.c", "mycode/protocol.c", "mycode/number_parse.c", "mycode/motor_params.c", "tests/motor_host/test_motion.c"],
    "sensorless": ["mycode/JerryFOC.c", "mycode/motor_control.c", "mycode/protocol.c", "mycode/number_parse.c", "mycode/st_hso.c", "mycode/motor_params.c", "tests/motor_host/test_sensorless.c"],
    "profiles": ["mycode/motor_params.c", "tests/motor_host/test_profiles.c"],
    "motor_math": ["mycode/motor_math.c", "tests/motor_host/test_motor_math.c"],
    "div64": ["mycode/compact_div64.c", "tests/motor_host/test_div64.c"],
    "float_widen": ["mycode/compact_div64.c", "tests/motor_host/test_float_widen.c"],
    "sixstep": ["mycode/motor_sixstep.c", "mycode/motor_params.c", "tests/motor_host/test_sixstep.c"],
    "sixstep_wave": ["mycode/motor_sixstep.c", "mycode/motor_params.c", "tests/motor_host/test_sixstep_wave.c"],
    "sixstep_plant": ["mycode/motor_sixstep.c", "mycode/motor_params.c", "tests/motor_host/test_sixstep_plant.c"],
    "sampling": ["mycode/motor_sampling.c", "mycode/motor_math.c", "mycode/motor_params.c", "tests/motor_host/test_sampling.c"],
    "encoder": ["mycode/MT6701.c", "mycode/w25q64.c", "mycode/app_storage.c", "mycode/motor_params.c", "mycode/angle_src_encoder.c", "mycode/motor_math.c", "tests/motor_host/test_encoder.c"],
    "foc": ["mycode/JerryFOC.c", "mycode/protocol.c", "mycode/number_parse.c", "mycode/motor_params.c", "tests/motor_host/test_foc.c"],
    "hso": ["mycode/st_hso.c", "mycode/motor_params.c", "tests/motor_host/test_hso.c"],
    "protocol": ["mycode/protocol.c", "mycode/number_parse.c", "mycode/motor_params.c", "tests/motor_host/test_protocol.c"],
    "telemetry": ["mycode/protocol.c", "mycode/number_parse.c", "mycode/motor_params.c", "tests/motor_host/test_telemetry.c"],
    "justfloat": ["mycode/protocol.c", "mycode/number_parse.c", "mycode/motor_params.c", "tests/motor_host/test_justfloat.c"],
    "number_parse": ["mycode/number_parse.c", "tests/motor_host/test_number_parse.c"],
}
selected=set(sys.argv[1:])
if selected-set(suites):
    raise SystemExit("Unknown suites: "+", ".join(sorted(selected-set(suites))))
if not selected or "sampling" in selected:
    subprocess.run([sys.executable,"tests/motor_host/check_key_boot.py"],cwd=repo,check=True)
for name, sources in suites.items():
    if selected and name not in selected: continue
    sources += ["tests/diag_stub.c"]
    if name in ("angle","encoder","detect","control","motion","sensorless"):
        sources += ["mycode/encoder_estimator.c"]
    if "mycode/protocol.c" in sources: sources += ["tests/motor_host/app_stub.c"]
    if name in ("detect","control","motion","sensorless"): sources += ["mycode/motor_sixstep.c","mycode/motor_detect.c"]
    if name in ("inverter","current","foc","protocol","telemetry","justfloat"):
        sources += ["tests/motor_host/control_stub.c"]
    if name in ("protocol","telemetry","justfloat"): sources += ["tests/motor_host/telemetry_stubs.c"]
    if name in ("inverter","current","foc","control","motion","sensorless","detect","hso","sixstep","sixstep_wave","sixstep_plant"):
        sources += ["mycode/motor_math.c"]
    command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1",
               "-Itests/motor_host", "-Imycode", "-IThirdParty/ST_HSO", *sources, "-lm", "-o", str(out / (name + ".exe"))]
    if name=="sampling": command.insert(1,"-DMOTOR_SAMPLING_HOST_HOOKS")
    if name=="encoder": command.insert(1,"-DMOTOR_ENCODER_HOST_HOOKS")
    if name=="inverter": command.insert(1,"-DMOTOR_INVERTER_HOST")
    if name in ("detect","control","motion","sensorless"): command.insert(1,"-DMOTOR_HOST_REAL_CONTROL")
    if name=="motor_math": command.insert(1,"-DMOTOR_MATH_CORDIC_HOST")
    subprocess.run(command, cwd=repo, check=True)
    subprocess.run([str(out / (name + ".exe"))], cwd=repo, check=True)
    if name=="control":
        for work in range(7):
            for reason in ("OUTER OVERRUN","OUTER GAP","OUTER STALE"):
                subprocess.run([str(out/"control.exe"),"outer-trip",str(work),reason],cwd=repo,check=True)
        subprocess.run([str(out/"control.exe"),"outer-exclude"],cwd=repo,check=True)
    if name=="sixstep_plant":
        for bus in (12,20):
            for direction in (-1,1):
                for inertia in (0.00001,0.00002,0.00004):
                    subprocess.run([str(out / "sixstep_plant.exe"),str(bus),str(direction),str(inertia)],cwd=repo,check=True)
        for scenario in ("stall","no-current","key","rc"):
            subprocess.run([str(out / "sixstep_plant.exe"),"20","1","0.00002",scenario],cwd=repo,check=True)
        for bus in (12,20):
            for direction in (-1,1):
                subprocess.run([str(out / "sixstep_plant.exe"),str(bus),str(direction),"0.00002","normal","10"],cwd=repo,check=True)
            for scenario,target in (("throttle","20"),("throttle-drop","25")):
                subprocess.run([str(out / "sixstep_plant.exe"),str(bus),"1","0.00002",scenario,target,"0.010"],cwd=repo,check=True)
        # Generic uses identical startup/control/protection policy regardless of
        # saved preset; physical R/L/poles remain solely in the independent plant.
        for bus,physical,inertia,flux in ((12,"2804",0.000005,0.0032),(12,"4015",0.00002,0.0075),(20,"4015",0.00002,0.0075)):
            for preset in ("2804","4015"):
                subprocess.run([str(out/"sixstep_plant.exe"),str(bus),"1",str(inertia),"throttle","30",str(flux),physical,"generic",preset],cwd=repo,check=True)
        for scenario in ("stall","no-current","key"):
            subprocess.run([str(out/"sixstep_plant.exe"),"12","1","0.00002",scenario,"30","0.0075","4015","generic"],cwd=repo,check=True)
        # Board feedback: the same 2804 starts with ESC 20% but not preset 20%.
        # Exercise both real paths against one physical model, plus preset Speed.
        for scenario,target in (("throttle","20"),("throttle","30"),("normal","30"),("normal","50")):
            for direction in ((-1,1) if scenario=="normal" else (1,)):
                subprocess.run([str(out/"sixstep_plant.exe"),"12",str(direction),"0.000005",scenario,target,"0.0032","2804"],cwd=repo,check=True)
        subprocess.run([str(out/"sixstep_plant.exe"),"12","1","0.000005","throttle","20","0.0032","2804","generic","2804"],cwd=repo,check=True)
        # Board-measured equivalent 4015 R/L/flux; current ADC quantization,
        # PWM ripple and rotor inertia stay independent of the controller.
        # R88 repeatedly reset its capture window at 20V in this model.
        for bus in (12,20):
            for inertia in (0.00002,0.00004,0.00008):
                for throttle in (30,75):
                    subprocess.run([str(out/"sixstep_plant.exe"),str(bus),"1",str(inertia),"throttle",str(throttle),"0.00929","4015","generic","4015","1.92","0.00044"],cwd=repo,check=True)
        for scenario in ("stall","no-current","key"):
            subprocess.run([str(out/"sixstep_plant.exe"),"20","1","0.00004",scenario,"75","0.00929","4015","generic","4015","1.92","0.00044"],cwd=repo,check=True)
    if name=="sampling":
        for scenario in ("diag-optional", "ibus", "ibus-wrap", "ibus-shared", "ibus-dac-set", "ibus-dac-start", "ibus-comp-start", "ibus-runtime", "ibus-high-start", "ibus-startup-pulse", "global", "global-wrap", "stop-source", "six-outputs"):
            subprocess.run([str(out / "sampling.exe"),scenario],cwd=repo,check=True)
    if name == "encoder":
        for scenario in ("calibration-direction", "missing", "timeout", "missing-wrap", "m1-recovery", "m2-recovery", "required-retry", "required-retry-wrap", "zero-m1", "zero-m2", "zero-poll", "hso-save", "busy-admission", "flash-init"):
            subprocess.run([str(out / "encoder.exe"), scenario], cwd=repo, check=True)
