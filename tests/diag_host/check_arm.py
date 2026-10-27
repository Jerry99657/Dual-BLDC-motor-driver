"""Execute actual linked Cortex-M4 exception instructions, not a C model.
Install unicorn/pyelftools outside sources or into ignored build/diag-testdeps.
"""
from pathlib import Path
import sys,struct
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'build/diag-testdeps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_MEM_WRITE,UC_HOOK_CODE
from unicorn.arm_const import *

def check(mode):
 with (root/f'build/{mode}/FOC_two.elf').open('rb') as f:
  elf=ELFFile(f)
  symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
  noinit=elf.get_section_by_name('.noinit');heap=elf.get_section_by_name('._user_heap_stack')
  assert noinit['sh_type']=='SHT_NOBITS' and noinit['sh_addr']>=symbols['_ebss']
  assert noinit['sh_addr']+noinit['sh_size']<=heap['sh_addr'] and heap['sh_addr']+heap['sh_size']<=0x20008000
  retained=symbols['retained'];stack=symbols['fault_stack'];length=noinit['sh_size']
  loads=[(p['p_vaddr'],p.data()) for p in elf.iter_segments() if p['p_type']=='PT_LOAD' and p['p_filesz']]
 def emulator():
  u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
  for address,size in [(0x08000000,0x20000),(0x20000000,0x8000),(0x40012000,0x2000),(0x40021000,0x1000),(0xe000e000,0x2000)]:u.mem_map(address,size)
  for address,data in loads:u.mem_write(address,data)
  return u
 def put(u,address,*values):u.mem_write(address,struct.pack('<'+'I'*len(values),*values))
 def get(u,address):return int.from_bytes(u.mem_read(address,4),'little')
 def init(u):
  u.reg_write(UC_ARM_REG_XPSR,0x01000000);u.reg_write(UC_ARM_REG_CONTROL,0)
  u.reg_write(UC_ARM_REG_SP,0x20007f00);u.reg_write(UC_ARM_REG_LR,0x08000001)
  u.emu_start(symbols['BoardDiag_Init']|1,0x08000000,count=30000)
  assert u.reg_read(UC_ARM_REG_PC)==0x08000000
 def crash(u,vector,kind,exc,sp,clock=0x2800,cfsr=0,extended=False,*,stage=7,
           hfsr=0x40000000,pc=0x08001235,lr=0x08000101,xpsr=0x21000000):
  put(u,0x40021060,clock)
  for base in (0x40012c00,0x40013400):put(u,base+0x44,0xdeadffff);put(u,base+0x20,0xbeefffff)
  put(u,0xe000ed28,cfsr,hfsr,0,0x12345678,0x20001234)
  put(u,symbols['g_boot_stage'],stage)
  if 0x20000000<=sp<=0x20007f00 and sp%4==0:
   put(u,sp+(72 if extended else 0),1,2,3,4,12,lr,pc,xpsr)
  u.reg_write(UC_ARM_REG_XPSR,0x01000003);u.reg_write(UC_ARM_REG_CONTROL,0)
  u.reg_write(UC_ARM_REG_MSP,sp if not(exc&4) else 0x20007e00)
  u.reg_write(UC_ARM_REG_PSP,sp if exc&4 else 0x20007d00)
  u.reg_write(UC_ARM_REG_LR,exc);u.reg_write(UC_ARM_REG_PRIMASK,0)
  writes=[];pcs=[];on_emergency=False
  def onwrite(uc,access,addr,size,value,_):
   if retained<=addr<retained+length:
    for bit,base in ((0x800,0x40012c00),(0x2000,0x40013400)):
     if clock&bit:assert get(uc,base+0x44)&0x8000==0 and get(uc,base+0x20)&0x555==0,'recorded before gates closed'
    writes.append(addr)
  def oncode(uc,address,size,_):
   nonlocal on_emergency
   if address==(symbols['BoardDiag_Fatal']&~1):on_emergency=True
   if on_emergency:assert stack<=uc.reg_read(UC_ARM_REG_MSP)<=stack+256,'emergency stack overflow'
   pcs.append(address)
   if len(pcs)>3 and pcs[-1]==pcs[-3] and pcs[-2]==pcs[-4]:uc.emu_stop()
  wh=u.hook_add(UC_HOOK_MEM_WRITE,onwrite);ch=u.hook_add(UC_HOOK_CODE,oncode)
  u.emu_start(symbols[vector]|1,0,count=40000)
  u.hook_del(wh);u.hook_del(ch)
  assert len(pcs)<40000 and u.reg_read(UC_ARM_REG_PRIMASK)==1
  assert stack<=u.reg_read(UC_ARM_REG_MSP)<=stack+256
  for bit,base in ((0x800,0x40012c00),(0x2000,0x40013400)):
   enabled=bool(clock&bit)
   assert get(u,base+0x44)==(0xdeadffff&~0x8000 if enabled else 0xdeadffff)
   assert get(u,base+0x20)==(0xbeefffff&~0x555 if enabled else 0xbeefffff)
  return struct.unpack('<11I',u.mem_read(retained+8,44)),writes
 for vector,kind in [('NMI_Handler',2),('HardFault_Handler',3),('MemManage_Handler',4),('BusFault_Handler',5),('UsageFault_Handler',6)]:
  for exc in (0xfffffff9,0xfffffffd,0xffffffed):
   for clock in (0,0x800,0x2000,0x2800):
    u=emulator();init(u)
    record,writes=crash(u,vector,kind,exc,0x20007800,clock,extended=not bool(exc&16))
    assert record==(kind,7,0x08001235,0x08000101,0x20007800,0x21000000,exc,0,0x40000000,0x20001234,0x12345678),record
    assert writes
    saved=bytes(u.mem_read(retained,length))
    # Model startup's actual .bss zero range. Emergency stack can be cleared,
    # retained section must survive. A normal boot validates but never erases it.
    start=symbols['_sbss'];u.mem_write(start,b'\0'*(symbols['_ebss']-start));init(u)
    assert bytes(u.mem_read(retained,length))==saved
    second,_=crash(u,'HardFault_Handler',3,exc,0x20007700,clock,extended=not bool(exc&16))
    assert second==record and bytes(u.mem_read(retained,length))==saved,'overwrote first CPU fault'
 for sp,cfsr in [(0,0),(0x20008000,0),(0x20007701,0),(0x20007800,0x1000),(0x20007800,0x10),(0x20007800,0x2000),(0x20007800,0x20)]:
  u=emulator() # Uninitialized retained RAM is also valid before BoardDiag_Init.
  r,_=crash(u,'HardFault_Handler',3,0xfffffff9,sp,cfsr=cfsr)
  assert r[0]==3 and r[2:4]==(0,0) and r[5]==0 and r[4]==sp
 # Execute the reported programming/debug event through the real HardFault
 # vector, including immediate gate shutdown and halt. Only its retained
 # loader record is removed by the next application initialization.
 loader=dict(stage=1430396978,hfsr=0x80000000,pc=0x20000000,lr=0x200007c1,xpsr=0x21000200)
 variants=[({},True),({'stage':0},True),({'stage':7},False),({'hfsr':0x40000000},False),
           ({'hfsr':0xc0000000},False),({'pc':0x08001234},False),({'pc':0x20000100},False),
           ({'cfsr':0x20000},False),({'sp':0x20007800},False),({'lr':0x08000101},False),
           ({'xpsr':0x21000203},False),({'exc':0xfffffffd},False)]
 for changes,discard in variants:
  args=loader|changes;sp=args.pop('sp',0x20000bd8);exc=args.pop('exc',0xfffffff9)
  u=emulator();init(u);record,writes=crash(u,'HardFault_Handler',3,exc,sp,**args)
  assert record[0]==3 and writes,'debug events still stop and record immediately'
  start=symbols['_sbss'];u.mem_write(start,b'\0'*(symbols['_ebss']-start));init(u)
  assert get(u,retained+8)==(0 if discard else 3),(changes,record)
  if discard:
   assert not any(u.mem_read(retained+8,44))
   subsequent,_=crash(u,'HardFault_Handler',3,0xfffffff9,0x20007800)
   assert subsequent[0:3]==(3,7,0x08001235),'loader event occupied real application crash slot'
  else: assert tuple(struct.unpack('<11I',u.mem_read(retained+8,44)))==record
 print(f'PASS: {mode} actual Thumb CPU vectors, MSP/PSP/extended FP frames, independent stack, early-init/invalid-stack faults, clock-gated gate shutdown before recording/halt, retained first crash, startup-zero boundary and strict loader-debug discrimination (139 cases)')
check('Debug')
if '--both' in sys.argv:check('Release')
