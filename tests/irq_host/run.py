"""Exercise the real dispatcher with unmodified ST LL function bodies.

Only hardware register reads/writes and application callbacks are modeled.
The generated headers live under build/; no vendor source is edited.
"""
from pathlib import Path
import re
import shutil
import subprocess

repo=Path(__file__).resolve().parents[2]
out=repo/'build/irq-validation'
inc=out/'include'
inc.mkdir(parents=True,exist_ok=True)
vendor=repo/'Drivers/STM32G4xx_HAL_Driver/Inc'
device=(repo/'Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g431xx.h').read_text()
hal=(vendor/'stm32g4xx_hal_adc.h').read_text()
constants=re.findall(r'^#define (?:ADC_(?:ISR|IER)_\w+|TIM_(?:SR|DIER)_\w+)\s+[^\n]+',device,re.M)
constants+=re.findall(r'^#define HAL_ADC_(?:STATE|ERROR)_\w+\s+[^\n]+',hal,re.M)
constants+=re.findall(r'^#define LL_ADC_FLAG_\w+\s+[^\n]+',(vendor/'stm32g4xx_ll_adc.h').read_text(),re.M)
constants=[line.split('/*',1)[0].rstrip() for line in constants]
(inc/'irq_constants.h').write_text('#pragma once\n'+'\n'.join(constants)+'\n')

functions={
    'exti':['LL_EXTI_IsActiveFlag_0_31','LL_EXTI_ClearFlag_0_31'],
    'gpio':['LL_GPIO_IsInputPinSet'],
    'tim':['LL_TIM_IsEnabledIT_UPDATE','LL_TIM_IsActiveFlag_UPDATE','LL_TIM_ClearFlag_UPDATE',
           'LL_TIM_IsEnabledIT_BRK','LL_TIM_IsActiveFlag_BRK','LL_TIM_IsActiveFlag_SYSBRK',
           'LL_TIM_IsActiveFlag_BRK2','LL_TIM_ClearFlag_BRK','LL_TIM_ClearFlag_SYSBRK','LL_TIM_ClearFlag_BRK2'],
    'adc':['LL_ADC_ClearFlag_OVR','LL_ADC_ClearFlag_JQOVF'],
}
for peripheral,names in functions.items():
    text=(vendor/('stm32g4xx_ll_'+peripheral+'.h')).read_text()
    snippets=[]
    for name in names:
        match=re.search(r'__STATIC_INLINE\s+[^;{}]+\b'+name+r'\([^;{}]*\)\s*\{[^{}]*\}',text)
        assert match,name
        snippets.append(match[0])
    if peripheral=='adc':
        for name in ('LL_ADC_ReadReg','LL_ADC_WriteReg'):
            match=re.search(r'^#define '+name+r'\([^\n]+',text,re.M)
            assert match and not match[0].endswith('\\'),name
            snippets.append(match[0])
    include='#include "main.h"\n'
    if peripheral not in ('exti','gpio'): include+='#include "'+peripheral+'.h"\n'
    (inc/('stm32g4xx_ll_'+peripheral+'.h')).write_text('#pragma once\n'+include+'\n'.join(snippets)+'\n')
compiler=shutil.which('gcc') or 'gcc'
subprocess.run([compiler,'-std=c11','-O1','-Wall','-Wextra','-Werror',
    '-I'+str(inc),'-Itests/irq_host','-Imycode','mycode/motor_irq.c',
    'tests/irq_host/test_irq.c','-o',str(out/'irq.exe')],cwd=repo,check=True)
subprocess.run([str(out/'irq.exe')],cwd=repo,check=True)
subprocess.run([compiler,'-std=c11','-O1','-Wall','-Wextra','-Werror',
    '-DMOTOR_IRQ_INTEGRATION','-include','irq_constants.h',
    '-I'+str(inc),'-Itests/motor_host','-Imycode','mycode/motor_irq.c',
    'mycode/motor_sampling.c','mycode/motor_params.c','tests/motor_host/test_sampling_irq.c',
    '-lm','-o',str(out/'sampling_irq.exe')],cwd=repo,check=True)
for scenario in ('paired','missing','adc-error','key','break','system-break','break2'):
    subprocess.run([str(out/'sampling_irq.exe'),scenario],cwd=repo,check=True)
