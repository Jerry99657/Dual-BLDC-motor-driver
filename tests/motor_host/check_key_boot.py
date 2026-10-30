"""Extract and execute the real pre-output main block; no mirrored startup.

Optional --baseline compiles the old main against the same RC/stop fixture to
reproduce KEY1 LOW at 0ms/S4 and 0/0 Now 1/0 before verifying the fix.
"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess

root=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--baseline',type=Path)
args=parser.parse_args()
out=root/'build/key-boot-validation';out.mkdir(exist_ok=True)

def check(path,settle):
    text=path.read_text(encoding='utf-8')
    block=re.search(r'/\* USER CODE BEGIN 2 \*/(.*?)/\* USER CODE END 2 \*/',text,re.S)[1]
    prefix=block.split('  /* 1.')[0]
    assert 'BoardDiag_LastCrash()->kind' in prefix and 'MotorSampling_IsGlobalStopped();' in prefix
    if settle:
        assert text.index('MX_GPIO_Init();') < text.index('HAL_Delay(30U);') < text.index('MT6701_StartContinuous();')
        assert block.index('MotorSampling_IsGlobalStopped();') < block.index('MotorControl_Init();') < block.index('MotorSampling_Start()')
    (out/'key_boot_prefix.inc').write_text(prefix,encoding='utf-8')
    exe=out/('key_boot.exe' if settle else 'key_boot_before.exe')
    subprocess.run([shutil.which('gcc'),'-std=c11','-Wall','-Wextra','-Werror','-O1',
        '-DMOTOR_SAMPLING_HOST_HOOKS',f'-DKEY_EXPECT_SETTLE={int(settle)}',
        '-Itests/motor_host','-Imycode','-I'+str(out),
        'mycode/motor_sampling.c','mycode/motor_math.c','mycode/motor_params.c','tests/diag_stub.c',
        'tests/motor_host/test_key_boot.c','-lm','-o',str(exe)],cwd=root,check=True)
    scenarios=('rc','rc-wrap','held','held-wrap','released','press','cpu','pre-edge','pending') if settle else ('rc','rc-wrap')
    for scenario in scenarios:
        subprocess.run([str(exe),scenario],cwd=root,check=True)
    print('PASS: '+('actual R103 main RC settle; no output/counter start during wait, live EXTI, held boot, short press/preexisting event/CPU/pending preserved, wrap and immediate dual-bridge runtime stop'
                    if settle else 'actual old main reproduces RC KEY1 LOW at 0ms/S4 and 0/0 Now 1/0, also across tick wrap'))

if args.baseline:check(args.baseline.resolve(),False)
check(root/'Core/Src/main.c',True)
