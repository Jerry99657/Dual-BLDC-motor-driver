"""Actual user SPI runtime + local ST LL bodies and G431 register constants.

Mapped low-address registers let the unmodified 32-bit ST DMA address helpers
run on a 64-bit host. Wrappers model only DR/OVR hardware side effects. No
serial port, board or vendor file is changed.
"""
from pathlib import Path
import argparse, re, shutil, subprocess, sys
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--baseline',type=Path,help='Compare DMA register work with an earlier user spi_port.c')
parser.add_argument('--lcd-baseline',type=Path,help='Compare LCD chunk progress while foreground service is unavailable')
args=parser.parse_args()

repo=Path(__file__).resolve().parents[2]
subprocess.run([sys.executable,'tests/spi_host/check_config.py'],cwd=repo,check=True)
out=repo/'build/spi-validation';inc=out/'include'
inc.mkdir(parents=True,exist_ok=True)
vendor=repo/'Drivers/STM32G4xx_HAL_Driver/Inc'
device=(repo/'Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g431xx.h').read_text()
hal=(vendor/'stm32g4xx_hal_spi.h').read_text()
constants=re.findall(r'^#define (?:SPI_(?:CR1|CR2|SR)_\w+|DMA_(?:CCR|CNDTR)_\w+)\s+[^\n]+',device,re.M)
constants+=re.findall(r'^#define HAL_SPI_ERROR_\w+\s+[^\n]+',hal,re.M)
dma=(vendor/'stm32g4xx_ll_dma.h').read_text()
constants+=re.findall(r'^#define LL_DMA_CHANNEL_[1-8]\s+[^\n]+',dma,re.M)
constants=[line.split('/*',1)[0].rstrip() for line in constants]
state=re.search(r'typedef enum\s*\{\s*HAL_SPI_STATE_RESET.*?\} HAL_SPI_StateTypeDef;',hal,re.S)[0]
(inc/'spi_constants.h').write_text('#pragma once\n'+'\n'.join(constants)+'\n'+state+'\n')
functions={
 'spi':['LL_SPI_Enable','LL_SPI_Disable','LL_SPI_EnableIT_ERR','LL_SPI_EnableDMAReq_RX',
        'LL_SPI_EnableDMAReq_TX','LL_SPI_IsActiveFlag_RXNE','LL_SPI_ReceiveData8',
        'LL_SPI_TransmitData8','LL_SPI_ClearFlag_OVR','LL_SPI_DMA_GetRegAddr'],
 'dma':['LL_DMA_DisableChannel','LL_DMA_EnableChannel','LL_DMA_DisableIT_TC',
        'LL_DMA_DisableIT_HT','LL_DMA_DisableIT_TE','LL_DMA_EnableIT_TE','LL_DMA_EnableIT_TC',
        'LL_DMA_SetPeriphAddress','LL_DMA_SetMemoryAddress','LL_DMA_SetDataLength'],
}
wrapped=['LL_SPI_ReceiveData8','LL_SPI_TransmitData8','LL_SPI_ClearFlag_OVR']
for peripheral,names in functions.items():
 text=(vendor/('stm32g4xx_ll_'+peripheral+'.h')).read_text()
 snippets=[]
 if peripheral=='dma':
  snippets.append(re.search(r'static const uint8_t CHANNEL_OFFSET_TAB\[\]\s*=\s*\{.*?\};',text,re.S)[0])
 for name in names:
  match=re.search(r'__STATIC_INLINE\s+[^;{}]+\b'+name+r'\([^;{}]*\)\s*\{[^{}]*\}',text)
  assert match,name
  body=match[0]
  if name in wrapped: body='#define '+name+' ST_'+name+'\n'+body+'\n#undef '+name
  snippets.append(body)
 for name in ('LL_'+peripheral.upper()+'_ReadReg','LL_'+peripheral.upper()+'_WriteReg'):
  snippets.append(re.search(r'^#define '+name+r'\([^\n]+',text,re.M)[0])
 if peripheral=='spi':
  snippets += [
   'static inline uint8_t LL_SPI_ReceiveData8(SPI_TypeDef *s) { uint8_t v=ST_LL_SPI_ReceiveData8(s);HostReceived(s);return v; }',
   'static inline void LL_SPI_TransmitData8(SPI_TypeDef *s,uint8_t v) { ST_LL_SPI_TransmitData8(s,v);HostTransmitted(s,v); }',
   'static inline void LL_SPI_ClearFlag_OVR(const SPI_TypeDef *s) { ST_LL_SPI_ClearFlag_OVR(s);HostClearOverrun(s); }',
  ]
 # Pointer width warnings belong to the unchanged 32-bit ST address helpers.
 (inc/('stm32g4xx_ll_'+peripheral+'.h')).write_text('#pragma once\n#include "spi.h"\n'
  '#pragma GCC diagnostic push\n#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"\n'
  '#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"\n'+'\n'.join(snippets)+'\n#pragma GCC diagnostic pop\n')
compiler=shutil.which('gcc') or 'gcc'
command=[compiler,'-std=c11','-O1','-Wall','-Wextra','-Werror',
 '-I'+str(inc),'-Itests/spi_host','-Imycode','mycode/spi_port.c',
 'tests/spi_host/test_spi.c','-o',str(out/'spi.exe')]
subprocess.run(command,cwd=repo,check=True)
after=subprocess.check_output([str(out/'spi.exe')],cwd=repo,text=True)
print(after,end='')
if args.lcd_baseline:
 baseline=command.copy();baseline.insert(1,'-DSPI_LCD_REFERENCE')
 baseline[baseline.index('mycode/spi_port.c')]=str(args.lcd_baseline.resolve())
 baseline[-1]=str(out/'spi-lcd-before.exe')
 subprocess.run(baseline,cwd=repo,check=True)
 before=subprocess.check_output([baseline[-1]],cwd=repo,text=True)
 (out/'lcd-before.txt').write_text(before);(out/'lcd-after.txt').write_text(after)
 old=int(re.search(r'LCD PIPE: (\d+)',before)[1]);new=int(re.search(r'LCD PIPE: (\d+)',after)[1])
 assert old==0 and new==20,(old,new)
 print('PASS: old LCD path stalls before first chunk callback; new path chains all 20 chunks with no foreground service')
if args.baseline:
 baseline=command.copy();baseline.insert(1,'-DSPI_DMA_REFERENCE');baseline[baseline.index('mycode/spi_port.c')]=str(args.baseline.resolve());baseline[-1]=str(out/'spi-before.exe')
 subprocess.run(baseline,cwd=repo,check=True)
 before=subprocess.check_output([baseline[-1]],cwd=repo,text=True)
 (out/'register-before.txt').write_text(before);(out/'register-after.txt').write_text(after)
 old=[tuple(map(int,m)) for m in re.findall(r'DMA WORK: (\d+) (\d+) (\d+)',before)]
 new=[tuple(map(int,m)) for m in re.findall(r'DMA WORK: (\d+) (\d+) (\d+)',after)]
 assert len(old)==len(new)==3
 for a,b in zip(old,new):
  assert a[0]==b[0] and a[1]>b[1] and a[2]>=b[2],(a,b)
  print(f'PASS: SPI{a[0]+1} complete transfer CCR writes {a[1]} -> {b[1]}, flag-clear writes {a[2]} -> {b[2]}; identical functional/remapping regressions')
