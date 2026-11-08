"""Run linked TIM2 + all six keys; modeled instruction counts are NOT board cycles."""
from pathlib import Path
import argparse,json,re,sys,struct
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tests/motor_host'))
from check_modulator_work_arm import Model,UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_PC
header=(ROOT/'Core/Inc/main.h').read_text()

def pin(name):
    port=re.search(r'#define\s+'+name+r'_GPIO_Port\s+GPIO([A-G])',header)[1]
    bit=int(re.search(r'#define\s+'+name+r'_Pin\s+GPIO_PIN_(\d+)',header)[1])
    return 0x48000010+0x400*(ord(port)-ord('A')),1<<bit

def run(path,case,legacy=False):
    m=Model(path);m.stub[m.symbols['BoardDiag_Timing']&~1]='BoardDiag_Timing'
    for port in range(7):m.put(0x48000010+0x400*port,0xffff)
    for i,name in enumerate(('KEY_Up','KEY_Down','KEY_OK','KEY_Left','KEY_Right','KEY2')):
        if case.get('raw',0)&(1<<i):
            address,bit=pin(name);m.put(address,m.get(address)&~bit)
    now=case.get('now',1000);m.put(m.symbols['uwTick'],now)
    m.put(0x4000000c,1);m.put(0x40000010,1)
    if legacy:
        address=m.symbols['s_capture']
        m.u.mem_write(address,struct.pack('<2HI5B',980,980,990,0,0,0,0,1))
    else:
        address=m.symbols['s_input'];initial=case.get('initial',[])
        data=struct.pack('<IH8B16B',case.get('pending_at',990),1,
            0,case.get('candidate',case.get('raw',0)),case.get('stable',0),63,
            case.get('long_sent',0),case.get('suppressed',0),0,len(initial),
            *(initial+[0]*(16-len(initial))))
        m.u.mem_write(address,data+bytes(2)) # InputState is 32 bytes on ARM.
        for i in range(6):
            m.u.mem_write(m.symbols['s_times']+8*i,struct.pack('<IHH',
                case.get('pressed_at',now)&0xffffffff,case.get('at',now-20)&65535,
                case.get('repeat_at',now+300)&65535))
    m.u.reg_write(UC_ARM_REG_SP,0x20007d00);m.u.reg_write(UC_ARM_REG_LR,0x08020001)
    m.u.emu_start(m.symbols['MotorIRQ_Tim2']|1,0x08020000,count=10000)
    assert m.u.reg_read(UC_ARM_REG_PC)==0x08020000
    assert m.hist['BoardDiag_Timing']==1
    assert not any(m.hist[n] for n in ('App_NavigationEvent','KK_UI_Update','KK_UI_InputAccepted'))
    if not legacy:
        state=bytes(m.u.mem_read(address,32));head,count=state[12:14]
        actual=[state[14+((head+i)&15)] for i in range(count)]
        assert actual==case['expected'],(case,actual)
        assert state[6]==case.get('raw',0)
    return m.count

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True,help='R128 ELF')
    args=p.parse_args()
    cases=[dict(expected=[]),dict(raw=1,candidate=0,expected=[]),
        dict(raw=4,at=995,expected=[]),dict(raw=4,suppressed=4,expected=[]),
        dict(raw=36,expected=[6]),dict(raw=63,expected=[6]),
        dict(raw=1,stable=1,pressed_at=600,repeat_at=999,expected=[9]),
        dict(raw=16,stable=16,pressed_at=400,expected=[37]),
        dict(stable=8,pressed_at=350,expected=[36,28]),
        dict(initial=[3],pending_at=0,now=1001,expected=[]),
        dict(raw=1,initial=[9]*16,expected=[]),
        dict(raw=32,initial=[9]*16,expected=[6]),
        dict(raw=4,at=0xfffffff0,now=16,expected=[3])]
    cases += [dict(raw=1<<i,expected=[i+1]) for i in range(6)]
    baseline=run(args.baseline,{},True);report={}
    for mode in ('Debug','Release'):
        counts=[run(ROOT/f'build/{mode}/FOC_two.elf',case) for case in cases]
        assert max(counts)<1500,(mode,counts) # Fixed-loop/queue work bound, not a cycle budget.
        report[mode]=dict(r128IdleIRQ=baseline,idleIRQ=counts[0],min=min(counts),max=max(counts),cases=len(cases))
        print('PASS:',mode,'actual TIM2 six-key recognizer:',report[mode],'; NOT board microseconds')
    out=ROOT/'build/keys-r129/input-work.json';out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report,indent=2))
