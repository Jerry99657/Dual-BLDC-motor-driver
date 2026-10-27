from pathlib import Path
import subprocess,shutil
root=Path(__file__).resolve().parents[2]
out=root/'build/diag-validation';out.mkdir(exist_ok=True)
subprocess.run([shutil.which('gcc'),'-std=c11','-O1','-Wall','-Wextra','-Werror','-Itests/motor_host','-Imycode','tests/diag_host/test_diag.c','mycode/ui_text.c','-lm','-o',str(out/'diag.exe')],cwd=root,check=True)
subprocess.run([str(out/'diag.exe')],check=True)
