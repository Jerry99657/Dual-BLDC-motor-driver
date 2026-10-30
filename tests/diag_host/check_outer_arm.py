"""Run the actual linked timing guard and SSI decoder in Cortex-M4 emulation.

Optional --baseline ELF measures R118 instruction counts. Counts are NOT board
cycles. Fault cleanup is hooked here; motor_host tests real two-axis cleanup.
"""
from pathlib import Path
import argparse
import random
import struct
import sys

root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'build/diag-testdeps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import *

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--baseline',type=Path)
args=parser.parse_args()

class Machine:
    def __init__(self,path):
        with path.open('rb') as f:
            elf=ELFFile(f)
            self.symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
            loads=[(p['p_vaddr'],p.data()) for p in elf.iter_segments() if p['p_type']=='PT_LOAD' and p['p_filesz']]
        self.u=u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
        for addr,size in [(0x08000000,0x21000),(0x20000000,0x8000),(0xe0001000,0x1000)]:
            u.mem_map(addr,size)
        for addr,data in loads:u.mem_write(addr,data)
        u.reg_write(UC_ARM_REG_XPSR,0x01000000)
        self.instructions=self.masked=self.max_masked=0
        self.faults=[]
        def instruction(uc,pc,size,user):
            self.instructions+=1
            self.masked=self.masked+1 if uc.reg_read(UC_ARM_REG_PRIMASK) else 0
            self.max_masked=max(self.max_masked,self.masked)
            if pc==(self.symbols.get('MotorControl_TimingFault',0)&~1):
                address=uc.reg_read(UC_ARM_REG_R0)
                self.faults.append(bytes(uc.mem_read(address,24)).split(b'\0')[0].decode())
                assert uc.reg_read(UC_ARM_REG_PRIMASK)==0,'guard did not restore PRIMASK before cleanup'
                uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
        u.hook_add(UC_HOOK_CODE,instruction)

    def call(self,name,*argv):
        u=self.u;self.instructions=self.masked=self.max_masked=0
        u.reg_write(UC_ARM_REG_SP,0x20007f00);u.reg_write(UC_ARM_REG_LR,0x08020001)
        for reg,arg in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2),argv):u.reg_write(reg,arg&0xffffffff)
        u.emu_start(self.symbols[name]|1,0x08020000,count=4000)
        assert u.reg_read(UC_ARM_REG_PC)==0x08020000,name
        assert u.reg_read(UC_ARM_REG_PRIMASK)==0,name
        return u.reg_read(UC_ARM_REG_R0),self.instructions,self.max_masked

    def arm(self,axis):
        # Stable packed prefix of the production controller (work/focus/drive,
        # then input[2]); verified by both real controller and linked tests.
        c=self.symbols['control']
        self.u.mem_write(c,bytes((0 if axis==0 else 1,0,0,1 if axis==0 else 0,1 if axis else 0)))
        self.u.mem_write(self.symbols['s_requested'],bytes((axis==0,axis==1)))

    def report(self,outer,start,duration):
        self.u.mem_write(0xe0001004,struct.pack('<I',(start+duration)&0xffffffff))
        return self.call('BoardDiag_Timing',outer,start,0)[1:]

    def decode(self,frame):
        a=0x20007000
        self.u.mem_write(a,frame.to_bytes(3,'big')+b'\0'*9)
        result,count,_=self.call('MT6701_Decode',a,a+8,a+10)
        return (result,bytes(self.u.mem_read(a+8,3))),count

def normal(path,axis):
    m=Machine(path);m.arm(axis);counts={0:[],1:[]};masked=[]
    for n in range(12):
        start=0xfff00000+n*170000
        count,critical=m.report(1,start,66810);counts[1].append(count);masked.append(critical)
        for j in range(1,10):
            count,critical=m.report(0,start+66810+j*17000-8500,8500)
            counts[0].append(count);masked.append(critical)
    assert not m.faults,m.faults
    return m,{('ADC','OUTER')[k]:max(v[1:]) for k,v in counts.items()},max(masked)

baseline=None
if args.baseline:
    baseline,counts,critical=normal(args.baseline,0)
    print(f'R118 reference: Thumb instruction maxima {counts}, longest PRIMASK run {critical} instructions')

rng=random.Random(119)
frames=[0,0xffffff,0x100,0x400,0xfffc00]+[rng.getrandbits(24) for _ in range(500)]
for mode in ('Debug','Release'):
    path=root/f'build/{mode}/FOC_two.elf'
    for axis in range(2):
        m,counts,critical=normal(path,axis)
        counters=struct.unpack('<9I',m.u.mem_read(m.symbols['timing'],36))
        assert counters[6:]==(0,0,0),counters
        m.report(0,0xfff00000+11*170000+66810+340000-8500,8500)
        assert m.faults==['OUTER STALE'],m.faults
        print(f'PASS: {mode} M{axis+1} actual Thumb normal/wrap + missing outer completion; '
              f'instruction maxima {counts}, longest PRIMASK run {critical} instructions')
    for what in ('long','late','short'):
        m=Machine(path);m.arm(0);m.report(1,0,4000)
        if what=='long':m.report(1,170000,475830)
        else:
            for n in range(1,4):m.report(1,n*(255001 if what=='late' else 84999),4000)
        assert m.faults==['OUTER OVERRUN' if what=='long' else 'OUTER GAP'],m.faults
    if baseline:
        current=Machine(path);delta=[]
        for frame in frames:
            expected,old=baseline.decode(frame);actual,new=current.decode(frame)
            assert actual==expected,(mode,hex(frame),actual,expected)
            delta.append(new-old)
        print(f'PASS: {mode} actual SSI decoder vs R118: {len(frames)} frames; instruction delta {min(delta)}..{max(delta)}')
