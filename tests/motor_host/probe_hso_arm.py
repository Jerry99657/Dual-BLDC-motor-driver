"""Run the linked official HSO ARM code on synthetic EMF via Unicorn.

Optional diagnostic, not a physical motor/ADC model or a timing measurement.
Only CORDIC cos/sin/polar are substituted by host math. Opaque HSO memory is
zero-initialized per HSO_init; every subsequent access uses public APIs.
Requires pyelftools and unicorn; accepts --python-path for local dependencies.
"""
import argparse
import math
from pathlib import Path
import struct
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--python-path', type=Path)
parser.add_argument('--elf', type=Path, default=Path('build/Debug/FOC_two.elf'))
parser.add_argument('--seconds', type=int, default=6)
parser.add_argument('--dc', type=float, nargs='+', default=[0, .1, .2])
parser.add_argument('--self-check', action='store_true', help='Run the fixed 15 Hz, 3 s DC-offset regression suite.')
args = parser.parse_args()
if args.python_path:
    sys.path.insert(0, str(args.python_path))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_CODE
from unicorn.arm_const import (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2,
                              UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_LR,
                              UC_ARM_REG_PC, UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC)

uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
uc.mem_map(0x08000000, 0x40000)
uc.mem_map(0x20000000, 0x20000)  # Extra scratch for the test, not target RAM sizing.
with args.elf.open('rb') as source:
    elf = ELFFile(source)
    symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    for segment in elf.iter_segments():
        if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
            uc.mem_write(segment['p_vaddr'], segment.data())
uc.reg_write(UC_ARM_REG_C1_C0_2, 0xf << 20)
uc.reg_write(UC_ARM_REG_FPEXC, 1 << 30)
Q = 1 << 30
handle, params, emf = 0x20010000, 0x20010400, 0x20010500
stop = 0x08030000


def q(value):
    return int(value * Q)


def signed(value):
    return value if value < (1 << 31) else value - (1 << 32)


def return_hook(machine, address, size, data):
    r0, r1, r2, r3 = (machine.reg_read(r) for r in
                      (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3))
    if data == 'cos':
        angle = signed(r0) / Q * 2 * math.pi
        machine.mem_write(r1, struct.pack('<ii', q(math.cos(angle)), q(math.sin(angle))))
    else:
        x, y = signed(r0) / Q, signed(r1) / Q
        angle = math.atan2(y, x) / (2 * math.pi) % 1
        machine.mem_write(r2, struct.pack('<i', q(angle)))
        machine.mem_write(r3, struct.pack('<i', q(math.hypot(x, y))))
    machine.reg_write(UC_ARM_REG_PC, machine.reg_read(UC_ARM_REG_LR))


for name, kind in [('FIXP30_CosSinPU', 'cos'), ('FIXP30_polar', 'polar')]:
    address = symbols[name] & ~1
    uc.hook_add(UC_HOOK_CODE, return_hook, kind, address, address)


def call(name, *values):
    uc.reg_write(UC_ARM_REG_SP, 0x2001fff0)
    for register, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), values):
        uc.reg_write(register, value & 0xffffffff)
    uc.reg_write(UC_ARM_REG_LR, stop | 1)
    uc.emu_start(symbols[name] | 1, stop, count=200000)
    assert uc.reg_read(UC_ARM_REG_PC) == stop, name
    return signed(uc.reg_read(UC_ARM_REG_R0))


flux, hz = 0.007491, 15.0
if args.self_check:
    args.seconds, args.dc = 3, [0, .08, .1, .2]
results = {}
for dc in args.dc:
    for offsets in (False, True):
        uc.mem_write(handle, bytes(240))  # Public HSO_Obj size, HSO_init semantics.
        uc.mem_write(params, struct.pack('<6f?3x3f', flux, 20, .0001, 72.6, 1000, 100, True, 10, 500, .5))
        call('HSO_setParams', handle, params)
        call('HSO_setAngle_pu', handle, 0)
        call('HSO_setFlag_TrackAngle', handle, 1)
        call('HSO_setFlag_EnableOffsetUpdate', handle, offsets)
        print(f'offset_limit={call("HSO_getOffsetLimit",handle)/Q*72.6:.5f} V', flush=True)
        errors, speeds, fluxes = [], [], []
        for i in range(args.seconds*10000):
            angle = (i + 1) * hz * 2 * math.pi / 10000
            amplitude = flux * hz * 2 * math.pi
            uc.mem_write(emf, struct.pack('<ii', q((-amplitude * math.sin(angle) + dc) / 72.6),
                                         q(amplitude * math.cos(angle) / 72.6)))
            call('HSO_run', handle, emf, 0)
            if i % 10 == 0:  # Diagnostic samples at 1 kHz, not a 10 kHz gate test.
                observed = call('HSO_getAngle_pu', handle) / Q * 2 * math.pi
                errors.append(math.degrees(math.remainder(observed - angle, 2 * math.pi)))
                speeds.append(call('HSO_getEmfSpeed_pu', handle) / Q * 1000)
                fluxes.append(call('HSO_getFluxAmpl_Wb', handle) / Q * 1000)
            if (i+1) % 10000 == 0:
                call('HSO_getOffset_ab_pu', emf+16, handle)
                ox, oy = struct.unpack('<ii', uc.mem_read(emf+16, 8))
                results[dc, offsets] = (max(abs(min(errors)),abs(max(errors))), min(speeds),max(speeds), ox/Q*72.6)
                print(f't={i//10000+1}s dc={dc:.2f} V offset_update={offsets}: '
                      f'angle {min(errors):.2f}..{max(errors):.2f} deg; '
                      f'frequency {min(speeds):.2f}..{max(speeds):.2f} Hz; '
                      f'flux {min(fluxes):.2f}..{max(fluxes):.2f} mWb; '
                      f'offset=({ox/Q*72.6:.4f},{oy/Q*72.6:.4f}) V', flush=True)
                errors.clear(); speeds.clear(); fluxes.clear()
if args.self_check:
    assert results[0,False][0] < .1 and results[0,True][0] < .1
    assert results[.08,False][0] > 15 and results[.08,True][0] < 10
    assert results[.1,True][0] < results[.1,False][0] / 2
    assert results[.2,True][0] > 90  # Large bias still not usable; no gate is bypassed here.
    assert abs(results[.08,True][3]) > .05
    print('PASS: official linked ARM HSO: clean input, disabled-offset distortion, bounded adaptation improvement and uncompensated large bias; idealized CORDIC math, no ADC/motor/timing model')
