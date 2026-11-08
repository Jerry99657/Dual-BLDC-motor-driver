"""Compare linked Capture work with modeled ADC registers and a fixed clock.

Uses the real Thumb sampling/current conversion/protection arithmetic. KEY1
input and the clock are boundary stubs; race/arming/fault behavior is covered
by sampling and IRQ integration suites. Instruction counts are not MCU times.
"""
import argparse
import json
from pathlib import Path
import struct
from check_modulator_work_arm import (
    Model, ROOT, ELFFile, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC, UC_HOOK_MEM_READ)

def offsets(path):
    with Path(path).open('rb') as file:
        dwarf=ELFFile(file).get_dwarf_info()
        for cu in dwarf.iter_CUs():
            for die in cu.iter_DIEs():
                name=die.attributes.get('DW_AT_name')
                if die.tag=='DW_TAG_variable' and name and name.value==b'acquisition':
                    kind=die.get_DIE_from_attribute('DW_AT_type')
                    if kind.tag=='DW_TAG_structure_type':
                        return {member.attributes['DW_AT_name'].value.decode():
                                member.attributes['DW_AT_data_member_location'].value
                                for member in kind.iter_children() if member.tag=='DW_TAG_member'}
    raise AssertionError('Acquisition DWARF layout unavailable')

def capture(path,delta,dual,layout=None):
    model=Model(path);fields=layout or offsets(path);base=model.symbols['acquisition']
    if layout:
        # Release omits user DWARF. Its source/ABI is identical to Debug;
        # additionally require the real linked acquisition size to agree.
        with Path(path).open('rb') as file:
            symbols=ELFFile(file).get_section_by_name('.symtab')
            assert symbols.get_symbol_by_name('acquisition')[0]['st_size']==252
    def fixed_counter(uc,access,address,size,value,data):
        if address==0xe0001004:model.put(address,50000)
    model.u.hook_add(UC_HOOK_MEM_READ,fixed_counter)
    # Reuse the model's scalar-return boundary hook. False means GetTick=0
    # and KEY1 released. Run bus_fresh itself rather than its optional stub.
    model.axis_ready=False;model.stub.pop(model.symbols['bus_fresh']&~1,None)
    for name in ('HAL_GetTick','MotorSampling_IsGlobalStopped'):
        model.stub[model.symbols[name]&~1]=name
    for name in ('running','power_enabled','bus_seen','bus_valid','reference_valid','ibus_started','ibus_ref_valid'):
        model.u.mem_write(base+fields[name],bytes([1]))
    model.put(base+fields['offset_count'],2000)
    model.u.mem_write(base+fields['offsets'],struct.pack('<5f',*([2048.]*5)))
    model.u.mem_write(model.symbols['s_monitored'],bytes([1,int(dual)]))
    model.put(model.symbols['hadc1'],0x50000000)
    model.put(model.symbols['hadc2'],0x50000100)
    model.put(0x50000100,1<<6)
    for adc in (0x50000000,0x50000100):
        for offset in (0x80,0x84,0x88,0x8c):model.put(adc+offset,2048+delta)
    model.u.reg_write(UC_ARM_REG_SP,0x20007d00)
    model.u.reg_write(UC_ARM_REG_LR,0x08020001)
    model.u.emu_start(model.symbols['MotorSampling_Capture']|1,0x08020000,count=20000)
    assert model.u.reg_read(UC_ARM_REG_PC)==0x08020000
    assert model.u.reg_read(UC_ARM_REG_R0)==1
    assert model.u.mem_read(base+fields['fault'],1)==b'\0'
    assert model.u.mem_read(base+fields['axis_fault'],2)==b'\0\0'
    result=bytes(model.u.mem_read(base,72))+bytes(model.u.mem_read(base+fields['energy'],8))
    return model.count,result,model.hist['HAL_GetTick']

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=ROOT/'build/sampling-reuse-validation/work.json')
    args=parser.parse_args();report={}
    debug_layout=offsets(ROOT/'build/Debug/FOC_two.elf')
    assert debug_layout==offsets(args.baseline),'Acquisition layout changed'
    for mode in ('Debug','Release'):
        cases=[]
        for dual in (False,True):
            for delta in (-200,-50,0,50,200):
                old,expected,old_clocks=capture(args.baseline,delta,dual)
                new,actual,new_clocks=capture(ROOT/f'build/{mode}/FOC_two.elf',delta,dual,debug_layout)
                assert actual==expected,(mode,dual,delta)
                assert new==old and old_clocks==new_clocks,(old,new,old_clocks,new_clocks)
                cases.append({'dual':dual,'delta':delta,'before':old,'after':new,
                              'clocksBefore':old_clocks,'clocksAfter':new_clocks})
        report[mode]=cases
        print('PASS:',mode,'10 linked Capture vectors match; instruction delta',
              min(row['before']-row['after'] for row in cases),'..',
              max(row['before']-row['after'] for row in cases),'; NOT board microseconds')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2))
