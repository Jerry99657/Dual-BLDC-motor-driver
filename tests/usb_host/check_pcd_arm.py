"""Run unchanged ST PCD + PMA + IRQ Thumb code with double-buffer support 1/0.

This is a USB register/host-packet model, not an electrical/enumeration test.
"""
from pathlib import Path
import json
import shutil
import struct
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'build/diag-testdeps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_MEM_WRITE,UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_PC,UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_XPSR

OUT=ROOT/'build/usb-validation/pcd-arm'
OUT.mkdir(parents=True,exist_ok=True)
compiler=shutil.which('arm-none-eabi-gcc')
assert compiler,'ARM compiler required'
includes=['Core/Inc','Drivers/STM32G4xx_HAL_Driver/Inc','Drivers/CMSIS/Device/ST/STM32G4xx/Include','Drivers/CMSIS/Include']
sources=['tests/usb_host/pcd_arm.c']+['Drivers/STM32G4xx_HAL_Driver/Src/'+n for n in ('stm32g4xx_hal_pcd.c','stm32g4xx_hal_pcd_ex.c','stm32g4xx_ll_usb.c')]
USB=0x40005C00
PMA=USB+0x400
STOP=0x0800F000

class Model:
    def __init__(self,path):
        with path.open('rb') as f:
            elf=ELFFile(f)
            self.symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
            loads=[(p['p_vaddr'],p.data()) for p in elf.iter_segments() if p['p_type']=='PT_LOAD' and p['p_filesz']]
        self.u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
        for address,size in [(0x08000000,0x10000),(0x20000000,0x8000),(0x40000000,0x30000),(0xe000e000,0x2000)]:self.u.mem_map(address,size)
        for address,data in loads:self.u.mem_write(address,data)
        self.pending=[]
        self.u.hook_add(UC_HOOK_MEM_WRITE,self.write)
        self.u.hook_add(UC_HOOK_CODE,self.instruction)
        self.u.reg_write(UC_ARM_REG_XPSR,0x01000000)
        self.call('Test_Init')

    def word(self,address):return int.from_bytes(self.u.mem_read(address,2),'little')
    def put(self,address,value):self.u.mem_write(address,struct.pack('<H',value))
    def events(self):return struct.unpack('<8I',self.u.mem_read(self.symbols['test_events'],32))
    def refresh(self):
        flags=self.word(USB+0x44)&0x7F00
        for endpoint in range(8):
            ep=self.word(USB+endpoint*4)
            if ep&0x8080:
                flags|=0x8000|endpoint|(0x10 if ep&0x8000 else 0)
                break
        self.put(USB+0x44,flags)
    def flush(self):
        for address,value in self.pending:self.put(address,value)
        self.pending.clear();self.refresh()
    def instruction(self,u,address,size,data):
        self.flush()
        if address==STOP:u.emu_stop()
    def write(self,u,access,address,size,value,data):
        if USB<=address<USB+32 and (address-USB)%4==0:
            old=self.word(address)
            # CTR flags clear on zero; DTOG/STAT toggle on one; EA/TYPE/KIND RW.
            new=(old&value&0x8080)|((old^value)&0x7070)|(value&0x070F)
            if new&0x8000:new|=old&0x0800
            self.pending.append((address,new))
        elif address==USB+0x44:
            self.pending.append((address,self.word(address)&value&0x7F00))
    def call(self,name,*args):
        self.u.reg_write(UC_ARM_REG_SP,0x20007F00)
        self.u.reg_write(UC_ARM_REG_LR,STOP|1)
        for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1),args):self.u.reg_write(reg,value)
        self.u.emu_start(self.symbols[name]|1,STOP,count=500000)
        self.flush()
        assert self.u.reg_read(UC_ARM_REG_PC)==STOP,(name,hex(self.u.reg_read(UC_ARM_REG_PC)))
        return self.u.reg_read(UC_ARM_REG_R0)
    def irq(self,endpoint,rx=False,setup=False):
        address=USB+endpoint*4
        self.put(address,self.word(address)|(0x8000 if rx else 0x80)|(0x800 if setup else 0))
        self.refresh();self.call('Test_Irq')
        assert not self.word(USB+0x44)&0x8000

def exercise(path):
    m=Model(path);trace=[]
    for length in (0,1,2,63,64,65,78,104,110,128,192):
        before=m.events()[0];assert m.call('Test_Tx',1,length)==0
        sent=bytearray()
        while m.events()[0]==before:
            count=m.word(PMA+8+2)
            assert count<=64
            sent+=m.u.mem_read(PMA+0xC0,count)
            m.irq(1)
            assert len(sent)<=length
        assert sent==bytes(i^0xA5 for i in range(length)),(length,sent)
        trace.append(('tx',length,bytes(sent).hex(),m.events()))
    for length in (0,1,2,63,64,65,78,110,128,192):
        before=m.events()[1];assert m.call('Test_Rx',1,length)==0
        payload=bytes((i*3+7)&255 for i in range(length));offset=0
        while m.events()[1]==before:
            part=payload[offset:offset+64]
            m.u.mem_write(PMA+0x110,part or b'\0')
            old=m.word(PMA+8+6);m.put(PMA+8+6,(old&0xFC00)|len(part))
            offset+=len(part);m.irq(1,rx=True)
            assert offset<=length
        assert offset==length and m.u.mem_read(m.symbols['test_rx'],length)==payload
        trace.append(('rx',length,payload.hex(),m.events()))
    m.call('Test_Stall',0x81);assert m.word(USB+4)&0x30==0x10
    m.call('Test_ClearStall',0x81);assert m.word(USB+4)&0x30!=0x10
    setup=bytes.fromhex('8006000100001200')
    m.u.mem_write(PMA+0x18,setup);m.put(PMA+6,8);m.irq(0,rx=True,setup=True)
    assert m.events()[4]==1 and m.u.mem_read(m.symbols['test_setup'],8)==setup
    before=m.events()[2];m.call('Test_Tx',0,0);m.irq(0);assert m.events()[2]==before+1
    for flag,index in ((0x800,6),(0x1000,7),(0x400,5)):
        before=m.events()[index];m.put(USB+0x44,flag);m.call('Test_Irq');assert m.events()[index]==before+1
    trace.append(('control-events',m.events()))
    return trace

traces=[]
for option in (1,0):
    image=OUT/f'pcd-{option}.elf'
    command=[compiler,'-mcpu=cortex-m4','-mthumb','-mfloat-abi=soft','-std=c11','-Os','-g','-fdata-sections','-ffunction-sections',
             '-DSTM32G431xx','-DUSE_HAL_DRIVER',f'-DUSE_USB_DOUBLE_BUFFER={option}U',*['-I'+x for x in includes],*sources,
             '-nostartfiles','--specs=nano.specs','-Wl,--gc-sections,-e,Test_Init,-Ttext=0x08000000,-Tdata=0x20000000',
             '-Wl,-u,Test_Tx,-u,Test_Rx,-u,Test_Irq,-u,Test_Stall,-u,Test_ClearStall','-o',str(image)]
    subprocess.run(command,cwd=ROOT,check=True)
    traces.append(exercise(image))
assert traces[0]==traces[1]
(OUT/'trace.json').write_text(json.dumps(traces[0],indent=2))
print('PASS: actual ST PCD/USB Thumb code support=1 vs 0: identical single-buffer IN/OUT 0..192 B, odd lengths, multi-packet, stall/clear, EP0 SETUP/ZLP, reset and suspend/resume')
