"""Compare actual R124/current control traces and count limit evaluations.

Instrumentation exists only in ignored host-test copies, never in firmware.
Requires --baseline path/to/motor_control.c to compare with an older revision.
"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/limits-validation'
OUT.mkdir(parents=True,exist_ok=True)
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--baseline',type=Path)
args=parser.parse_args()

def run(label,path):
    source=path.read_text(encoding='utf-8')
    for name,counter in [('speed_limit','test_speed_evals'),('position_speed','test_position_evals')]:
        marker=f'static float {name}(MotorAxis axis)\n{{'
        assert source.count(marker)==1,(path,name)
        source=source.replace(marker,marker+f'\n    ++{counter};')
    copy=OUT/f'{label}.c'
    copy.write_text('unsigned test_speed_evals,test_position_evals;\n'+source,encoding='utf-8')
    sources=['mycode/JerryFOC.c',str(copy),'mycode/protocol.c','mycode/number_parse.c',
        'mycode/motor_params.c','tests/motor_host/test_limits.c','tests/diag_stub.c',
        'tests/motor_host/app_stub.c','mycode/motor_sixstep.c','mycode/motor_detect.c',
        'mycode/motor_math.c','mycode/encoder_estimator.c']
    exe=OUT/f'{label}.exe'
    subprocess.run([shutil.which('gcc'),'-std=c11','-Wall','-Wextra','-Werror','-O1',
        '-DMOTOR_HOST_REAL_CONTROL','-Itests/motor_host','-Imycode','-IThirdParty/ST_HSO',
        *sources,'-lm','-o',str(exe)],cwd=ROOT,check=True)
    result=subprocess.check_output([str(exe)],cwd=ROOT,text=True)
    (OUT/f'{label}.txt').write_text(result,encoding='utf-8')
    values=[line for line in result.splitlines() if line.startswith(('VALUE ','CASES '))]
    counts=[tuple(map(int,m)) for m in re.findall(r'^EVAL (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)$',result,re.M)]
    assert len(counts)==5120
    return values,counts

actual,counts=run('after',ROOT/'mycode/motor_control.c')
if args.baseline:
    expected,old=run('before',args.baseline.resolve())
    assert actual==expected,'Control output/settings trace changed'
    assert len(old)==len(counts)
    for a,b in zip(old,counts):
        assert a[:5]==b[:5] and a[5]>=b[5] and a[6]>=b[6],(a,b)
    for field,name in [(5,'speed_limit'),(6,'position_speed')]:
        before=sum(row[field] for row in old);after=sum(row[field] for row in counts)
        assert before>=after,(name,before,after)
        print(f'PASS: {name} evaluations {before} -> {after} across 5120 real commands')
print('PASS: both axes/2804+4015, zero/invalid/default/extended setting fields, signed zero, +/-targets, fallback/custom position speed including M2 0.1 floor, live cap publication; output bits/settings match baseline when provided')
