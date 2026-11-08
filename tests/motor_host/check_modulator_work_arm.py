"""Actual linked modulator work comparison, with modeled bus/readiness/CORDIC.

Can check the modulator alone or complete PI/FF/modulation/anti-windup call.
Does not run the complete ADC ISR and does NOT model board cycle timings.
Readiness and parameter lookup are boundary stubs. DMA/outer protection has
separate integration tests. The acquisition test layout is the existing R123
layout (frame at 0, currents at 16, bus voltage at 144); revisit if it changes.
"""
from pathlib import Path
import sys,struct,bisect,math,collections
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'build/diag-testdeps'))
from elftools.elf.elffile import ELFFile
from unicorn import *
from unicorn.arm_const import *

class Model:
 def __init__(self,path):
  with Path(path).open('rb') as f:
   elf=ELFFile(f);self.symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
   funcs=sorted((s['st_value']&~1,s['st_size'],s.name) for s in elf.get_section_by_name('.symtab').iter_symbols() if s['st_info']['type']=='STT_FUNC' and s['st_size'])
   self.starts=[x[0] for x in funcs];self.funcs=funcs
   loads=[(s['p_vaddr'],s.data()) for s in elf.iter_segments() if s['p_type']=='PT_LOAD' and s['p_filesz']]
  self.u=u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
  for a,n in [(0x08000000,0x21000),(0x20000000,0x8000),(0x40000000,0x30000),(0x48000000,0x10000),(0x50000000,0x10000),(0xe0000000,0x100000)]:u.mem_map(a,n)
  for a,b in loads:u.mem_write(a,b)
  u.reg_write(UC_ARM_REG_XPSR,0x01000000);u.reg_write(UC_ARM_REG_C1_C0_2,0xf<<20);u.reg_write(UC_ARM_REG_FPEXC,1<<30)
  self.count=0;self.hist=collections.Counter();self.phase=0;self.nr=0;self.nw=0
  self.hso=False; self.drive=True; self.axis_ready=True; self.drive_calls=0
  self.stub={self.symbols[n]&~1:n for n in ['bus_fresh','MotorParams_GetAxis','MotorSampling_GetBusVoltage','MotorSampling_IsAxisReady','JerryFOC_isDriveReady','MotorControl_HsoAllowed'] if n in self.symbols}
  def code(uc,pc,size,data):
   self.count+=1;i=bisect.bisect_right(self.starts,pc)-1
   if i>=0:
    a,n,name=self.funcs[i]
    if pc<a+n:self.hist[name]+=1
   if pc in self.stub:
    name=self.stub[pc]
    if name=='MotorParams_GetAxis':uc.reg_write(UC_ARM_REG_R0,self.get(self.symbols['axis_motors']+4*uc.reg_read(UC_ARM_REG_R0)))
    elif name=='MotorSampling_GetBusVoltage':uc.reg_write(UC_ARM_REG_S0,int.from_bytes(struct.pack('<f',self.bus),'little'))
    elif name=='MotorControl_HsoAllowed':uc.reg_write(UC_ARM_REG_R0,int(self.hso))
    elif name=='JerryFOC_isDriveReady':
     self.drive_calls+=1;uc.reg_write(UC_ARM_REG_R0,int(self.drive if self.drive!=2 else self.drive_calls==1))
    else:uc.reg_write(UC_ARM_REG_R0,int(self.axis_ready))
    uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
  def read(uc,access,addr,size,value,data):
   if addr==0x40020c00:self.put(addr,self.get(addr)|0x80000000)
   if addr==0x40020c08:
    value=math.cos(self.phase) if self.nr==0 else math.sin(self.phase);self.nr=(self.nr+1)%2
    self.put(addr,int(value*2147483647)&0xffffffff)
   if addr==0xe0001004:self.put(addr,100000+self.count*2)
  def write(uc,access,addr,size,value,data):
   if addr==0x40020c00:self.nw=self.nr=0
   if addr==0x40020c04:
    if self.nw==0:self.phase=(value if value<0x80000000 else value-0x100000000)*math.pi/2147483648
    self.nw+=1
  u.hook_add(UC_HOOK_CODE,code);u.hook_add(UC_HOOK_MEM_READ,read);u.hook_add(UC_HOOK_MEM_WRITE,write)
  for name,ptr in [('htim1',0x40012c00),('htim8',0x40013400)]:
   self.put(self.symbols[name],ptr)
   for off,val in [(0,0x60),(0x24,3000),(0x28,0),(0x2c,8500),(0x44,0x8000)]:self.put(ptr+off,val)
  if 'axis_motors' in self.symbols and 'MOTOR_4015' in self.symbols:self.put(self.symbols['axis_motors']+4,self.symbols['MOTOR_4015'])
 def get(self,a):return int.from_bytes(self.u.mem_read(a,4),'little')
 def put(self,a,b):self.u.mem_write(a,struct.pack('<I',b&0xffffffff))
 def call(self,axis,bus=20,angle=1,speed=100,current=0,scope="modulator",gain=1):
  self.put(self.symbols['SystemCoreClock'],170000000)
  frame=self.symbols['acquisition']
  self.u.mem_write(frame+16,struct.pack('<5f',current,-0.4*current,-0.6*current,current,-0.4*current))
  self.bus=bus;self.put(self.symbols['acquisition']+144,int.from_bytes(struct.pack('<f',bus),'little'));u=self.u;self.count=0;self.hist.clear();self.drive_calls=0
  u.reg_write(UC_ARM_REG_SP,0x20007d00);u.reg_write(UC_ARM_REG_LR,0x08020001)
  for r,v in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3),(axis,0x20007e00,0x20007e04,1)):u.reg_write(r,v)
  for r,v in zip((UC_ARM_REG_S0,UC_ARM_REG_S1,UC_ARM_REG_S2,UC_ARM_REG_S3),(2.,.2,angle,speed)):u.reg_write(r,int.from_bytes(struct.pack('<f',v),'little'))
  # R126 adds a fifth core argument to the private modulator. Older images
  # ignore this word; public APIs and the current-control ABI are unchanged.
  self.put(0x20007d00,self.get(self.symbols['axis_motors']+4*axis))
  entry='axis_voltage'
  if scope=='current':
   entry='JerryFOC_CurrentControl';loops=0x20007b00
   self.u.mem_write(loops,bytes(132))
   for off in (64,96):self.u.mem_write(loops+off,struct.pack('<8f',.6,500,0,0,6,0,0,0))
   self.u.mem_write(loops+128,struct.pack('<f',gain))
   u.reg_write(UC_ARM_REG_R0,axis);u.reg_write(UC_ARM_REG_R1,loops)
   for i,value in enumerate((angle,speed,.1,.6,current,-.4*current)):
    u.reg_write(UC_ARM_REG_S0+i,int.from_bytes(struct.pack('<f',value),'little'))
  u.emu_start(self.symbols[entry]|1,0x08020000,count=50000)
  assert u.reg_read(UC_ARM_REG_PC)==0x08020000,hex(u.reg_read(UC_ARM_REG_PC))
  assert u.reg_read(UC_ARM_REG_PRIMASK)==0
  return self.count,self.hist.copy()

if __name__=='__main__':
 import argparse,json
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--baseline',type=Path,help='Previous firmware ELF for identical-vector/work comparison')
 parser.add_argument('--scope',choices=('modulator','current'),default='modulator')
 parser.add_argument('--output',type=Path,default=ROOT/'build/outer-stale-r124/modulator-work.json')
 args=parser.parse_args();report={}
 for mode in ('Debug','Release'):
  current=Model(ROOT/f'build/{mode}/FOC_two.elf');before=Model(args.baseline) if args.baseline else None
  old_counts=[];new_counts=[];nominal={}
  cases=0
  for axis in range(2):
   for bus in (0,12,20):
    for angle in (0,.7,2.8,5.4):
     for amps in (-.4,-.05,0,.4):
      states=((False,True,True),(False,False,True),(False,2,True),(False,True,False),(True,True,True)) if args.scope=='current' else ((False,True,True),)
      for hso,drive,axis_ready in states:
       for model in (current,before):
        if model:model.hso=hso;model.drive=drive;model.axis_ready=axis_ready
       count,hist=current.call(axis,bus,angle,100,amps,args.scope);new_counts.append(count);cases+=1
       timer=0x40012c00 if axis==0 else 0x40013400
       def result(model):
        extra=bytes(model.u.mem_read(0x20007b00,132)) if args.scope=='current' else bytes(model.u.mem_read(0x20007e00,8))
        return bytes(model.u.mem_read(timer+0x34,12))+extra
       actual=result(current)
       assert all(current.get(timer+0x34+4*i)<=8500 for i in range(3))
       if before:
        old,old_hist=before.call(axis,bus,angle,100,amps,args.scope);old_counts.append(old)
        if bus==20 and angle==.7 and amps==.4 and not hso and drive==True and axis_ready:
         nominal['M'+str(axis+1)]={'before':old,'after':count,'gettersBefore':{n:old_hist[n] for n in before.stub.values()},'gettersAfter':{n:hist[n] for n in current.stub.values()}}
        expected=result(before)
        assert actual==expected,(mode,axis,bus,angle,amps,hso,drive,axis_ready,actual.hex(),expected.hex())
        assert current.drive_calls==before.drive_calls,(current.drive_calls,before.drive_calls)
  report[mode]={'nominal':nominal,'cases':len(new_counts),'instructionsMin':min(new_counts),'instructionsMax':max(new_counts)}
  if before:
   report[mode].update(baselineMin=min(old_counts),baselineMax=max(old_counts),savedMin=min(a-b for a,b in zip(old_counts,new_counts)),savedMax=max(a-b for a,b in zip(old_counts,new_counts)),savedTotal=sum(old_counts)-sum(new_counts))
  print('PASS:',mode,args.scope+' with modeled peripherals:',report[mode],'; instruction counts are NOT board microseconds')
 args.output.parent.mkdir(exist_ok=True,parents=True)
 args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
