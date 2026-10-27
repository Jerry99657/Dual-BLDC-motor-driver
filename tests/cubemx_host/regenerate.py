"""Run installed CubeMX in scratch projects; never regenerate the workspace.

python tests/cubemx_host/regenerate.py --cubemx D:/Tools/STM32CubeMX_6.17
Requires the project's installed STM32Cube FW_G4 package. Output is retained
under ignored build/cubemx-regeneration, including the native generated files.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root / 'scripts'))
from check_cubemx import audit, value, clean

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cubemx', type=Path, required=True)
parser.add_argument('--verify-only', action='store_true', help='Recheck existing generated scratch results without rerunning CubeMX')
args = parser.parse_args()
out = root / 'build/cubemx-regeneration'
out.mkdir(parents=True, exist_ok=True)
ioc = (root/'FOC_two.ioc').read_text(encoding='utf-8')
script = []
projects = {}
for name in ('baseline', 'uart-change'):
    project = out / name / 'FOC_two'
    project.mkdir(parents=True, exist_ok=True)
    projects[name] = project
    if args.verify_only:
        continue
    for folder in ('Core', 'USB_Device'):
        shutil.copytree(root/folder, project/folder, dirs_exist_ok=True)
    config = ioc
    if name == 'uart-change':
        settings = {'BaudRate': '230400', 'StopBits': 'STOPBITS_2',
                    'OverSampling': 'UART_OVERSAMPLING_8', 'ClockPrescaler': 'PRESCALER_DIV2'}
        for key, val in settings.items():
            assert f'USART1.{key}=' not in config, 'Update fixture for new project defaults'
            config += f'USART1.{key}={val}\n'
        config = config.replace('USART1.IPParameters=VirtualMode-Asynchronous',
                                'USART1.IPParameters=VirtualMode-Asynchronous,'+','.join(settings))
        config = config.replace('NVIC.USART1_IRQn=true\\:4\\:', 'NVIC.USART1_IRQn=true\\:5\\:')
        # UART clock selection is a generated RCC setting, not a user constant.
        config += 'RCC.USART1CLockSelection=RCC_USART1CLKSOURCE_SYSCLK\n'
        config = re.sub(r'RCC.IPParameters=([^\n]+)', r'RCC.IPParameters=\1,USART1CLockSelection', config)
    (project/'FOC_two.ioc').write_text(config, encoding='utf-8')
    # Deliberately NO setDriver command: the saved IOC must retain LL selection.
    script += [f'config load "{(project/"FOC_two.ioc").as_posix()}"',
               f'project path "{project.parent.as_posix()}"', 'project generate']
script += ['exit']
script_file = out/'generate.txt'
if not args.verify_only:
    script_file.write_text('\n'.join(script)+'\n', encoding='utf-8')
java = args.cubemx/'jre/bin/java.exe'
if not args.verify_only:
    with (out/'generate.log').open('w', encoding='utf-8') as log:
        subprocess.run([str(java), '-jar', str(args.cubemx/'STM32CubeMX.exe'),
                        '-q', str(script_file)], cwd=root, stdout=log, stderr=subprocess.STDOUT,
                       timeout=300, check=True)

baseline = projects['baseline']
# Every generated Core source/header must survive a normal repeat generation.
for source in (root/'Core').rglob('*'):
    if source.suffix in ('.h', '.c'):
        name = source.relative_to(root)
        assert source.read_bytes() == (baseline/name).read_bytes(), str(name)
assert (baseline/'FOC_two.ioc').read_text(encoding='utf-8') == ioc
audit(baseline)
changed = projects['uart-change']
audit(changed)
uart = clean((changed/'Core/Src/usart.c').read_text(encoding='utf-8'))
for field, val in {'BaudRate': '230400', 'PrescalerValue': 'LL_USART_PRESCALER_DIV2',
                   'StopBits': 'LL_USART_STOPBITS_2', 'OverSampling': 'LL_USART_OVERSAMPLING_8'}.items():
    assert value(uart, 'USART_InitStruct.'+field) == val, field
assert value(uart, 'PeriphClkInit.Usart1ClockSelection') == 'RCC_USART1CLKSOURCE_SYSCLK'
assert 'NVIC_GetPriorityGrouping(),5, 0)' in uart

# Compile both generated variants with the actual ARM compile configuration.
entry = next(e for e in json.loads((root/'build/Debug/compile_commands.json').read_text())
             if Path(e['file']).name == 'usart.c')
base_args = shlex.split(entry['command'].replace('\\', '/'))
source_at, output_at = base_args.index('-c')+1, base_args.index('-o')+1
for name, project in projects.items():
    command = base_args.copy()
    command[0] = command[0].replace('\\', '/')
    command[source_at] = str(project/'Core/Src/usart.c')
    command[output_at] = str(out/(name+'.o'))
    command.insert(1, '-I'+str(project/'Core/Inc'))
    subprocess.run(command, cwd=root, check=True)

# The size/source-boundary audit uses hashes from this independent generation.
sources = ('Core/Src/usart.c', 'Core/Inc/usart.h', 'Core/Inc/main.h',
           'Core/Inc/stm32g4xx_hal_conf.h', 'Core/Src/stm32g4xx_it.c', 'Core/Src/spi.c')
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
verified = {'ioc': sha(root/'FOC_two.ioc'), 'native_project': baseline.relative_to(root).as_posix(),
            'sources': {name: sha(baseline/name) for name in sources},
            'official_sources': {p.relative_to(baseline).as_posix(): sha(p)
                                 for p in (baseline/'Drivers/STM32G4xx_HAL_Driver/Src').glob('*.c')}}
(out/'verified.json').write_text(json.dumps(verified, indent=2), encoding='utf-8')
print('PASS: CubeMX repeat generation retains every Core C/header and saved IOC; '
      'LL UART baud/prescaler/stop/oversampling/clock/NVIC changes pass audit and ARM compilation')
