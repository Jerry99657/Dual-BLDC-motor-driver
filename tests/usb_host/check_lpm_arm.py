"""Compare actual USB Device/CDC control processing with two generated LPM headers.

The USBD_LL transport is a recording stub; PCD/PMA has a separate Thumb test.
"""
from pathlib import Path
import argparse
import json
import shutil
import struct
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'build/diag-testdeps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_PC,UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_XPSR

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--baseline',type=Path,required=True,help='Previous generated usbd_conf.h')
args=parser.parse_args()
out=ROOT/'build/usb-validation/lpm-arm';out.mkdir(parents=True,exist_ok=True)
compiler=shutil.which('arm-none-eabi-gcc');assert compiler
includes=['USB_Device/App','Core/Inc','Drivers/STM32G4xx_HAL_Driver/Inc',
          'Drivers/CMSIS/Device/ST/STM32G4xx/Include','Drivers/CMSIS/Include',
          'Middlewares/ST/STM32_USB_Device_Library/Core/Inc',
          'Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc']
sources=['tests/usb_host/lpm_arm.c','USB_Device/App/usbd_desc.c']
sources+=['Middlewares/ST/STM32_USB_Device_Library/Core/Src/'+x for x in ('usbd_core.c','usbd_ctlreq.c','usbd_ioreq.c')]
sources+=['Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c']
STOP=0x0800F000

def exercise(path):
    with path.open('rb') as f:
        elf=ELFFile(f)
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
        loads=[(p['p_vaddr'],p.data()) for p in elf.iter_segments() if p['p_type']=='PT_LOAD' and p['p_filesz']]
    u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    u.mem_map(0x08000000,0x10000);u.mem_map(0x20000000,0x8000)
    u.mem_map(0x1FFF7000,0x1000) # Fixed factory UID register fixture for generated descriptors.
    u.mem_write(0x1FFF7590,struct.pack("<3I",0x12345678,0x23456789,0x3456789A))
    for address,data in loads:u.mem_write(address,data)
    u.reg_write(UC_ARM_REG_XPSR,0x01000000)
    def call(name,*values):
        u.reg_write(UC_ARM_REG_SP,0x20007F00);u.reg_write(UC_ARM_REG_LR,STOP|1)
        for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1),values):u.reg_write(reg,value)
        u.emu_start(symbols[name]|1,STOP,count=300000)
        assert u.reg_read(UC_ARM_REG_PC)==STOP,name
        return u.reg_read(UC_ARM_REG_R0)
    call('Test_Init');trace=[]
    requests=[('device',0x80,6,0x100,0,18),('configuration-head',0x80,6,0x200,0,9),
              ('configuration',0x80,6,0x200,0,255)]
    requests += [(f'string-{n}',0x80,6,0x300+n,0x409,255) for n in range(7)]
    requests += [('unsupported-BOS',0x80,6,0xF00,0,255),('address',0,5,5,0,0),
                 ('configure',0,9,1,0,0),('get-configuration',0x80,8,0,0,1),
                 ('get-status',0x80,0,0,0,2),('CDC-line-state',0x21,0x22,1,0,0)]
    for name,bm,req,value,index,length in requests:
        lo,hi=struct.unpack('<2I',struct.pack('<BBHHH',bm,req,value,index,length))
        call('Test_Request',lo,hi)
        meta=struct.unpack('<5I',u.mem_read(symbols['test_meta'],20))
        data=bytes(u.mem_read(symbols['test_reply'],meta[0])) if meta[0]<=512 else b''
        trace.append((name,meta,data.hex()))
        if name in ('unsupported-BOS','string-6'):assert meta[1]==3
        else:assert meta[1]==0,(name,meta)
        if name=='device':assert len(data)==18 and data[2:4]==b'\0\2'
        if name=='configuration':assert len(data)==67 and data[1]==2
        if name=='address':assert meta[2]==5 and meta[3]==2
        if name=='configure':assert meta[3]==3
        if name=='get-configuration':assert data==b'\1'
    state=call('Test_SuspendResume');assert state==0x030403,hex(state)
    trace.append(('suspend-resume',state))
    return trace

traces=[]
for label,header in [('before',args.baseline.resolve()),('after',ROOT/'USB_Device/Target/usbd_conf.h')]:
    image=out/(label+'.elf')
    command=[compiler,'-mcpu=cortex-m4','-mthumb','-mfloat-abi=soft','-std=c11','-Os','-g',
             '-fdata-sections','-ffunction-sections','-DSTM32G431xx','-DUSE_HAL_DRIVER','-DUSE_USB_DOUBLE_BUFFER=0U',
             '-I'+str(header.parent),*['-I'+x for x in includes],*sources,'-nostartfiles','--specs=nano.specs',
             '-Wl,--gc-sections,-e,Test_Init,-Ttext=0x08000000,-Tdata=0x20000000',
             '-Wl,-u,Test_Request,-u,Test_SuspendResume','-o',str(image)]
    subprocess.run(command,cwd=ROOT,check=True)
    traces.append(exercise(image))
assert traces[0]==traces[1],'LPM option changed ordinary USB control responses'
(out/'trace.json').write_text(json.dumps(traces[0],indent=2))
print('PASS: actual USB Device/CDC + generated descriptors, previous/current LPM headers: identical device/configuration/6 strings, unsupported string/BOS stall, address, CDC configuration/line state, get-status and ordinary suspend/resume')
