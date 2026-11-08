"""Compare real FOC admission/gates with R125; instrumentation is test-only."""
import argparse
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/foc-reuse-validation'
OUT.mkdir(parents=True,exist_ok=True)
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--baseline',type=Path,required=True)
args=parser.parse_args()
def run(label,path):
    text=path.read_text(encoding='utf-8')
    marker='uint8_t JerryFOC_isDriveReady(void)\n{'
    assert text.count(marker)==1
    text='unsigned test_drive_reads;\n'+text.replace(marker,marker+'\n    ++test_drive_reads;')
    source=OUT/(label+'.c');source.write_text(text,encoding='utf-8')
    exe=OUT/(label+'.exe')
    subprocess.run([shutil.which('gcc'),'-std=c11','-Wall','-Wextra','-Werror','-O1',
        '-Itests/motor_host','-Imycode','-IThirdParty/ST_HSO',str(source),
        'mycode/protocol.c','mycode/number_parse.c','mycode/motor_params.c',
        'tests/motor_host/test_foc_reuse.c','tests/diag_stub.c',
        'tests/motor_host/app_stub.c','tests/motor_host/control_stub.c',
        'mycode/motor_math.c','-lm','-o',str(exe)],cwd=ROOT,check=True)
    result=subprocess.check_output([str(exe)],cwd=ROOT,text=True)
    (OUT/(label+'.txt')).write_text(result)
    values=[row for row in result.splitlines() if row.startswith(('VALUE','CASES'))]
    counts=[list(map(int,row.split()[1:])) for row in result.splitlines() if row.startswith('EVAL')]
    assert len(counts)==192
    return values,counts
expected,before=run('before',args.baseline.resolve())
actual,after=run('after',ROOT/'mycode/JerryFOC.c')
assert actual==expected,'FOC output/current/target trace changed'
saved=0
for old,new in zip(before,after):
    assert old[:4]==new[:4]
    profile,loop,flags,voltage,reads=old
    admitted=bool(flags&4 and flags&8)
    assert reads-new[-1]==int(admitted),(old,new)
    if flags==15:
        # Early admission + live CurrentControl/encoder voltage checks.
        # This legacy fixture additionally adapts IsAxisReady through
        # isDriveReady; that extra adapter is identical in both images.
        assert new[-1]==(4 if loop else 3),(old,new)
    saved+=reads-new[-1]
print('PASS: 192 real FOC traces match for 2804/4015, current/voltage mode, VBUS 0/12/20, idle/unready/unhealthy/unaligned; removed',saved,'duplicate admission checks; later live gates retained')
