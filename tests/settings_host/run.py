from pathlib import Path
import shutil,subprocess
root=Path(__file__).resolve().parents[2]
out=root/'build/settings-validation';out.mkdir(exist_ok=True)
source=root/'mycode/app_settings.c'
text=source.read_text(encoding='utf-8')
assert not any(name in text for name in ('kk_ui','KK_UI','OLED','ui_app.h','ui_app.c','malloc('))
subprocess.run([shutil.which('gcc'),'-std=c11','-Wall','-Wextra','-Werror','-O1',
 '-Itests/motor_host','-Imycode',str(source),'mycode/motor_params.c','tests/settings_host/test_settings.c',
 '-lm','-o',str(out/'settings.exe')],cwd=root,check=True)
subprocess.run([str(out/'settings.exe')],cwd=root,check=True)
