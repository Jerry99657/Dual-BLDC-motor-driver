from pathlib import Path
import re,shutil,subprocess
root=Path(__file__).resolve().parents[2]
out=root/'build/adc-port-validation';out.mkdir(exist_ok=True)
inc=root/'Drivers/STM32G4xx_HAL_Driver/Inc'
sources=[root/'Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g431xx.h',inc/'stm32g4xx_hal_adc.h',inc/'stm32g4xx_hal_dma.h']
constants=[]
for p in sources:
 for line in p.read_text().splitlines():
  if re.match(r'#define (?:ADC_(?:CR|CFGR|ISR|IER)_\w+|HAL_ADC_(?:STATE|ERROR)_\w+|DMA_CCR_\w+|HAL_DMA_ERROR_\w+)\s',line):
   constants.append(line.split('/*',1)[0].rstrip())
(out/'adc_constants.h').write_text('#pragma once\n'+'\n'.join(constants)+'\n')
subprocess.run([shutil.which('gcc'),'-std=c11','-O1','-Wall','-Wextra','-Werror',
 '-I'+str(out),'-Itests/adc_port_host','-Imycode','mycode/adc_port.c',
 'tests/adc_port_host/test_port.c','-o',str(out/'port.exe')],cwd=root,check=True)
subprocess.run([str(out/'port.exe')],cwd=root,check=True)
