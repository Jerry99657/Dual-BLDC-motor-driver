from pathlib import Path
import argparse,subprocess,shutil
parser=argparse.ArgumentParser()
parser.add_argument('--formatter-baseline',type=Path,help='Previous board_diag.c for exact row comparison')
args=parser.parse_args()
root=Path(__file__).resolve().parents[2]
out=root/'build/diag-validation';out.mkdir(exist_ok=True)
command=[shutil.which('gcc'),'-std=c11','-O1','-Wall','-Wextra','-Werror','-Itests/motor_host','-Imycode','tests/diag_host/test_diag.c','mycode/ui_text.c','-lm','-o',str(out/'diag.exe')]
subprocess.run(command,cwd=root,check=True)
subprocess.run([str(out/'diag.exe')],check=True)
if args.formatter_baseline:
 baseline=command.copy()
 baseline.insert(1,'-DBOARD_DIAG_TEST_SOURCE="'+args.formatter_baseline.resolve().as_posix()+'"')
 baseline[-1]=str(out/'diag-baseline.exe')
 subprocess.run(baseline,cwd=root,check=True)
 before=subprocess.check_output([baseline[-1],'formatter'])
 after=subprocess.check_output([str(out/'diag.exe'),'formatter'])
 (out/'formatter-before.txt').write_bytes(before)
 (out/'formatter-after.txt').write_bytes(after)
 assert before==after,'Diagnostic row formatting changed'
 print(f'PASS: {len(after.splitlines())} real diagnostic rows match baseline, including signed/extreme/invalid floats, counters, CPU, empty records and both axes')
