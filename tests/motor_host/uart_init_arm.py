"""Check actual linked USART initialization even when LTO inlines it.

Clock/other peripheral handshakes are stubbed. The USART init instructions,
register writes, baud division and enable/ack loop execute in Thumb emulation.
"""
import re
import struct
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'build/diag-testdeps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE,UC_HOOK_MEM_READ
from unicorn.arm_const import UC_ARM_REG_PC,UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_XPSR,UC_ARM_REG_C1_C0_2,UC_ARM_REG_FPEXC

def verify_uart_init(elf_path,disassembly):
    with Path(elf_path).open('rb') as f:
        elf=ELFFile(f)
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
        loads=[(p['p_vaddr'],p.data()) for p in elf.iter_segments() if p['p_type']=='PT_LOAD' and p['p_filesz']]
    sites={}
    for line in disassembly.splitlines():
        m=re.match(r'\s*([0-9a-f]+):\s+([0-9a-f ]+)\s+blx?(?:\.[nw])?\s+[0-9a-f]+\s+<([^>]+)>',line)
        if m:sites[int(m[1],16)]=(len(m[2].replace(' ',''))//2,m[3])
    u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    for address,size in [(0x08000000,0x20000),(0x20000000,0x8000),(0x40000000,0x30000),
                         (0x48000000,0x3000),(0xe000e000,0x2000)]:u.mem_map(address,size)
    for address,data in loads:u.mem_write(address,data)
    def put(address,value):u.mem_write(address,struct.pack('<I',value))
    def get(address):return int.from_bytes(u.mem_read(address,4),'little')
    put(0x40021008,3<<2) # Generated 170 MHz HSE/PLL; no APB divider.
    put(0x4002100c,3|(1<<4)|(85<<8)|(1<<24))
    put(0x4001381c,(1<<21)|(1<<22)) # Hardware ack is ready after UE.
    if 'SystemCoreClock' in symbols:put(symbols['SystemCoreClock'],170000000)
    u.reg_write(UC_ARM_REG_XPSR,0x01000000)
    u.reg_write(UC_ARM_REG_SP,0x20007f00)
    u.reg_write(UC_ARM_REG_LR,0x08000001)
    u.reg_write(UC_ARM_REG_C1_C0_2,0xf00000);u.reg_write(UC_ARM_REG_FPEXC,0x40000000)
    acknowledgements=[]
    def read(uc,access,address,size,value,data):
        if address==0x4001381c and get(0x40013800)&1:acknowledgements.append(uc.reg_read(UC_ARM_REG_PC))
    def code(uc,address,size,data):
        if len(acknowledgements)>=2:uc.emu_stop();return
        if address not in sites:return
        length,name=sites[address]
        if name.startswith(('MX_USART1_UART_Init','LL_USART_','LL_RCC_','LL_GPIO_Init',
                            'HAL_GPIO_Init','HAL_NVIC_','__NVIC_','NVIC_EncodePriority')):return
        if name.startswith('Error_Handler'):raise AssertionError('Initialization took Error_Handler')
        r0,r1,r2=(uc.reg_read(r) for r in (UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2))
        if name=='memset':uc.mem_write(r0,bytes([r1&255])*r2);result=r0
        elif name=='memcpy':uc.mem_write(r0,bytes(uc.mem_read(r1,r2)));result=r0
        elif name.startswith('RCC_GetSystemClockFreq'):result=170000000
        elif name.startswith('HAL_') and not name.startswith(('HAL_Delay','HAL_COMP_MspInit','HAL_DAC_MspInit','HAL_TIM_MspPostInit')):result=0
        else:result=r0
        uc.reg_write(UC_ARM_REG_R0,result)
        uc.reg_write(UC_ARM_REG_LR,(address+length)|1)
        uc.reg_write(UC_ARM_REG_PC,(address+length)|1)
    u.hook_add(UC_HOOK_MEM_READ,read);u.hook_add(UC_HOOK_CODE,code)
    u.emu_start(symbols['main']|1,0,count=100000)
    assert len(acknowledgements)>=2,'USART enable/ack path was not reached from main'
    generated=(ROOT/'Core/Src/usart.c').read_text(encoding='utf-8')
    baud=int(re.search(r'USART_InitStruct\.BaudRate\s*=\s*(\d+)',generated)[1])
    assert get(0x4001380c)==(170000000+baud//2)//baud,('BRR',get(0x4001380c),baud)
    assert get(0x40013800)==13 and get(0x40013804)==0 and get(0x40013808)==0,'UE/8N1/TX/RX/FIFO/flow configuration'
    assert get(0x4001382c)==0,'Generated DIV1 prescaler'
