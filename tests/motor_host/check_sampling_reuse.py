"""Compare actual old/new sampling traces and clock reads; no firmware hooks."""
import argparse
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/sampling-reuse-validation'
OUT.mkdir(parents=True,exist_ok=True)
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--baseline',type=Path,required=True)
args=parser.parse_args()
fixture=(ROOT/'tests/motor_host/test_sampling.c').read_text()
assert fixture.count('uint32_t HAL_GetTick(void) {')==1
fixture=fixture.replace('uint32_t HAL_GetTick(void) {','unsigned test_tick_reads;\nuint32_t HAL_GetTick(void) {\n    ++test_tick_reads;')
(OUT/'test_sampling.c').write_text(fixture)
# Quote includes resolve relative to the including file. Put both fixture
# and driver in the test directory so the instrumented copy is selected.
shutil.copyfile(ROOT/'tests/motor_host/test_sampling_reuse.c',OUT/'driver.c')
def run(label,path):
    exe=OUT/(label+'.exe')
    subprocess.run([shutil.which('gcc'),'-std=c11','-Wall','-Wextra','-Werror','-O1',
        '-DMOTOR_SAMPLING_HOST_HOOKS','-Itests/motor_host','-Imycode',
        str(path),'mycode/motor_math.c','mycode/motor_params.c',str(OUT/'driver.c'),
        'tests/diag_stub.c','-lm','-o',str(exe)],cwd=ROOT,check=True)
    result=subprocess.check_output([str(exe)],cwd=ROOT,text=True)
    (OUT/(label+'.txt')).write_text(result)
    values=[s for s in result.splitlines() if s.startswith('VALUE')]
    reads=[int(s.split()[-1]) for s in result.splitlines() if s.startswith('EVAL')]
    assert len(values)==len(reads)==14
    return values,reads
expected,before=run('before',args.baseline.resolve())
actual,after=run('after',ROOT/'mycode/motor_sampling.c')
assert actual==expected,'Published samples/faults/gates changed'
assert before==after,(before,after)
print('PASS: 14 real capture traces and clock counts match; no cached sampling ages;',sum(before),'->',sum(after),'reads; immediate hardware gate regressions remain in sampling suite')
