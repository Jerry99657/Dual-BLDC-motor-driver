"""Compare real linked user min/max with the unchanged target libm.

Instruction counts are executed Thumb instructions, not hardware time.
Uses the optional Unicorn/pyelftools installed under build/diag-testdeps.
"""
from pathlib import Path
import random
import struct
import sys

root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'build/diag-testdeps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import (UC_ARM_REG_SP,UC_ARM_REG_LR,UC_ARM_REG_PC,
    UC_ARM_REG_C1_C0_2,UC_ARM_REG_FPEXC,UC_ARM_REG_XPSR,UC_ARM_REG_S0,UC_ARM_REG_S1)


def check(mode):
    with (root/f'build/{mode}/FOC_two.elf').open('rb') as stream:
        elf=ELFFile(stream)
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
        loads=[(p['p_vaddr'],p.data()) for p in elf.iter_segments()
               if p['p_type']=='PT_LOAD' and p['p_filesz']]
    u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
    u.mem_map(0x08000000,0x21000);u.mem_map(0x20000000,0x8000)
    for address,data in loads:u.mem_write(address,data)
    u.reg_write(UC_ARM_REG_C1_C0_2,0xf<<20);u.reg_write(UC_ARM_REG_FPEXC,1<<30)
    u.reg_write(UC_ARM_REG_XPSR,0x01000000)
    count=[0]
    def instruction(*args):count[0]+=1
    u.hook_add(UC_HOOK_CODE,instruction)

    def call(name,a,b):
        count[0]=0
        u.reg_write(UC_ARM_REG_SP,0x20007f00);u.reg_write(UC_ARM_REG_LR,0x08020001)
        u.reg_write(UC_ARM_REG_S0,a);u.reg_write(UC_ARM_REG_S1,b)
        u.emu_start(symbols[name]|1,0x08020000,count=1000)
        assert u.reg_read(UC_ARM_REG_PC)==0x08020000,name
        return u.reg_read(UC_ARM_REG_S0),count[0]

    boundaries=[0,0x80000000,1,0x80000001,0x7fffff,0x807fffff,0x800000,0x80800000,
                0x3f800000,0xbf800000,0x7f7fffff,0xff7fffff,0x7f800000,0xff800000,
                0x7fc12345,0xffc23456,0x7f800001,0xff800001]
    pairs=[(a,b) for a in boundaries for b in boundaries]
    rng=random.Random(118)
    pairs += [(rng.getrandbits(32),rng.getrandbits(32)) for _ in range(2000)]
    for a,b in pairs:
        for user,lib in [('MotorMath_Min','fminf'),('MotorMath_Max','fmaxf')]:
            actual,_=call(user,a,b);expected,_=call(lib,a,b)
            nan=lambda x:(x&0x7fffffff)>0x7f800000
            assert (nan(actual) and nan(expected)) or actual==expected,(user,hex(a),hex(b),hex(actual),hex(expected))
    counts={name:call(name,0x3f800000,0x40000000)[1]
            for name in ('MotorMath_Min','fminf','MotorMath_Max','fmaxf')}
    assert counts['MotorMath_Min']<counts['fminf']
    assert counts['MotorMath_Max']<counts['fmaxf']
    print(f'PASS: {mode} actual Thumb min/max vs unchanged target libm: {len(pairs)} pairs, '
          f'signed zero, both NaN classes, infinity and subnormal; instruction counts {counts}')


for mode in ('Debug','Release'):check(mode)
