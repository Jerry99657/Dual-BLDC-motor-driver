"""Actual USART transport with unmodified ST LL function bodies/register bits.

The host models W1C, RDR consumption and transmitter completion. No vendor
files are changed and no serial port is opened.
"""
from pathlib import Path
import re, shutil, subprocess

repo=Path(__file__).resolve().parents[2]
out=repo/'build/uart-validation'; inc=out/'include'
inc.mkdir(parents=True,exist_ok=True)
device=(repo/'Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g431xx.h').read_text()
constants=re.findall(r'^#define USART_(?:CR1|CR3|ISR|ICR|RDR)_\w+\s+[^\n]+',device,re.M)
constants=[line.split('/*',1)[0].rstrip() for line in constants]
(inc/'uart_constants.h').write_text('#pragma once\n'+'\n'.join(constants)+'\n')
text=(repo/'Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_ll_usart.h').read_text()
names=['LL_USART_DisableIT_TXE_TXFNF','LL_USART_DisableIT_TC','LL_USART_ClearFlag_TC',
       'LL_USART_EnableIT_ERROR','LL_USART_EnableIT_RXNE_RXFNE','LL_USART_IsEnabledIT_RXNE_RXFNE',
       'LL_USART_EnableIT_TXE_TXFNF','LL_USART_ReceiveData8','LL_USART_TransmitData8','LL_USART_EnableIT_TC']
snippets=[]
for name in names:
    match=re.search(r'__STATIC_INLINE\s+[^;{}]+\b'+name+r'\([^;{}]*\)\s*\{[^{}]*\}',text)
    assert match,name
    snippets.append(match[0])
for name in ('LL_USART_ReadReg','LL_USART_WriteReg'):
    match=re.search(r'^#define '+name+r'\([^\n]+',text,re.M)
    assert match and not match[0].endswith('\\'),name
    snippets.append(match[0])
(inc/'stm32g4xx_ll_usart.h').write_text('#pragma once\n#include "main.h"\n'+'\n'.join(snippets)+'\n')
compiler=shutil.which('gcc') or 'gcc'
subprocess.run([compiler,'-std=c11','-O1','-Wall','-Wextra','-Werror',
    '-I'+str(inc),'-Itests/uart_host','-Imycode','mycode/uart_port.c',
    'tests/uart_host/test_uart.c','-o',str(out/'uart.exe')],cwd=repo,check=True)
subprocess.run([str(out/'uart.exe')],cwd=repo,check=True)
