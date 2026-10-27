from pathlib import Path
import shutil
import subprocess
import re
import sys

root=Path(__file__).resolve().parents[2]
output=root/'build/ws2812-validation'
output.mkdir(parents=True,exist_ok=True)
inc=output/'include';inc.mkdir(exist_ok=True)
vendor=root/'Drivers/STM32G4xx_HAL_Driver/Inc'
device=(root/'Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g431xx.h').read_text()
constants=re.findall(r'^#define (?:DMA_(?:CCR|CNDTR)_\w+|TIM_(?:CR1|CR2|DIER|EGR|CCER|CCMR1|SR)_\w+)\s+[^\n]+',device,re.M)
constants+=re.findall(r'^#define DMA_MDATAALIGN_(?:HALFWORD|WORD)\s+[^\n]+',(vendor/'stm32g4xx_hal_dma.h').read_text(),re.M)
constants+=re.findall(r'^#define LL_TIM_CHANNEL_CH1\s+[^\n]+',(vendor/'stm32g4xx_ll_tim.h').read_text(),re.M)
(inc/'ws_constants.h').write_text('#pragma once\n'+'\n'.join(s.split('/*',1)[0].rstrip() for s in constants)+'\n')
registers=[]
for name in ('TIM_TypeDef','DMA_TypeDef','DMA_Channel_TypeDef'):
    registers.append(re.search(r'typedef struct\s*\{[^{}]*\}\s*'+name+r';',device,re.S)[0])
(inc/'ws_registers.h').write_text('#pragma once\n'+'\n'.join(registers)+'\n')
functions={
 'tim':['LL_TIM_DisableDMAReq_CC1','LL_TIM_DisableCounter','LL_TIM_OC_SetCompareCH1',
        'LL_TIM_GenerateEvent_UPDATE','LL_TIM_CC_DisableChannel','LL_TIM_SetCounter',
        'LL_TIM_EnableDMAReq_CC1','LL_TIM_CC_EnableChannel','LL_TIM_EnableCounter'],
 'dma':['LL_DMA_DisableChannel','LL_DMA_SetPeriphAddress','LL_DMA_SetMemoryAddress',
        'LL_DMA_SetDataLength','LL_DMA_EnableChannel'],
}
for peripheral,names in functions.items():
    text=(vendor/('stm32g4xx_ll_'+peripheral+'.h')).read_text()
    snippets=[]
    if peripheral=='dma':
        snippets.append(re.search(r'static const uint8_t CHANNEL_OFFSET_TAB\[\]\s*=\s*\{.*?\};',text,re.S)[0])
    for name in names:
        match=re.search(r'__STATIC_INLINE\s+[^;{}]+\b'+name+r'\([^;{}]*\)\s*\{[^{}]*\}',text)
        assert match,name
        snippets.append(match[0])
    if peripheral=='dma':
        for name in ('LL_DMA_ReadReg','LL_DMA_WriteReg'):
            snippets.append(re.search(r'^#define '+name+r'\([^\n]+',text,re.M)[0])
    (inc/('stm32g4xx_ll_'+peripheral+'.h')).write_text('#pragma once\n#include "tim.h"\n'
       '#pragma GCC diagnostic push\n#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"\n'
       '#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"\n'+'\n'.join(snippets)+'\n#pragma GCC diagnostic pop\n')
options=['-Wl,--image-base,0x10000000','-Wl,--disable-dynamicbase'] if sys.platform=='win32' else ['-no-pie']
subprocess.run([shutil.which('gcc') or 'gcc','-std=c11','-O1','-Wall','-Wextra','-Werror',
                *options,'-I'+str(inc),'-Itests/ws2812_host','-Imycode','mycode/ws2812.c','tests/ws2812_host/test_ws2812.c',
                '-o',str(output/'ws2812.exe')],cwd=root,check=True)
subprocess.run([str(output/'ws2812.exe')],check=True)
