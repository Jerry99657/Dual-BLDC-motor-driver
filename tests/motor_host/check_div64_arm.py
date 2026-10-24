"""Optional linked Thumb ABI check; requires unicorn and pyelftools on PYTHONPATH."""
import random, struct
from pathlib import Path
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS
from unicorn.arm_const import *
repo=Path(__file__).resolve().parents[2]
with (repo/'build/Debug/FOC_two.elf').open('rb') as f:
 elf=ELFFile(f); syms={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
 uc=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
 uc.mem_map(0x08000000,0x20000);uc.mem_map(0x20000000,0x10000)
 for segment in elf.iter_segments():
  if segment['p_type']=='PT_LOAD' and segment['p_filesz']:
   uc.mem_write(segment['p_vaddr'],segment.data())
mask=(1<<64)-1
regs=[UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3]
def call(name,a,b):
 uc.reg_write(UC_ARM_REG_SP,0x20007000)
 uc.reg_write(UC_ARM_REG_LR,0x08000001)
 for reg,v in zip(regs,[a&0xffffffff,(a>>32)&0xffffffff,b&0xffffffff,(b>>32)&0xffffffff]): uc.reg_write(reg,v)
 uc.mem_write(0x20007000,struct.pack('<I',0x20001000))
 uc.emu_start(syms[name]|1,0x08000000,count=10000)
 assert uc.reg_read(UC_ARM_REG_PC)==0x08000000
 q=uc.reg_read(regs[0])|(uc.reg_read(regs[1])<<32)
 r=int.from_bytes(uc.mem_read(0x20001000,8),'little') if name=='__udivmoddi4' else uc.reg_read(regs[2])|(uc.reg_read(regs[3])<<32)
 return q,r
rng=random.Random(70)
edges=[0,1,2,2**32-1,2**32,2**63-1,2**63,mask]
pairs=[(a,b) for a in edges for b in edges if b]+[(rng.getrandbits(64),rng.getrandbits(64)|1) for _ in range(500)]
for a,b in pairs:
 for fn in ('__udivmoddi4','__aeabi_uldivmod'):
  assert call(fn,a,b)==divmod(a,b),(fn,a,b,call(fn,a,b),divmod(a,b))
for _ in range(500):
 a=rng.randrange(-(1<<63),(1<<63)); b=rng.randrange(-(1<<63),(1<<63)) or 1
 q=abs(a)//abs(b); q=-q if (a<0)!=(b<0) else q; r=a-q*b
 assert call('__aeabi_ldivmod',a&mask,b&mask)==(q&mask,r&mask)
print('PASS: actual linked Cortex-M4 Thumb divider + signed/unsigned GCC ABI wrappers, full-range edges and 1500+ randomized calls')
