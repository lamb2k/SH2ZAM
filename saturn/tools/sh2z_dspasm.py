#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 SH2ZAM Contributors.
"""Minimal SCU DSP assembler for SH2ZAM kernels.

Covers the subset SH2ZAM programs use, written from the public SCU DSP
instruction format. One instruction per line; the parallel fields of an
operation word are separated by '|'. Labels end with ':'; ';' starts a comment.

  operation word:  [ALU] | [X-bus] | [Y-bus] | [D1-bus]   (any subset, any order)
     ALU:   NOP AND OR XOR ADD SUB AD2 SR RR SL RL RL8
     X-bus: MOV [s],X   MOV MUL,P   MOV [s],P
     Y-bus: MOV [s],Y   CLR A   MOV ALU,A   MOV [s],A
     D1:    MOV #imm8,[d]   MOV [s],[d]     ([s] may also be ALL or ALH)
  MVI #imm,[d]          25-bit signed immediate ([d]: MC0-3 RX PL RA0 WA0 LOP PC)
  DMA D0,[MCn|PRG],#n   DMA [MCn],D0,#n     (DMAH: hold the D0 address)
  JMP label | JMP cond,label   (cond: Z NZ S NS C NC T0 NT0 ZS NZS)
  LPS   BTM   END   ENDI

Usage: sh2z_dspasm.py IN.dsp OUT.h SYMBOL   writes a C array of 32-bit words.
"""
import re
import sys

ALU = {'NOP': 0, 'AND': 1, 'OR': 2, 'XOR': 3, 'ADD': 4, 'SUB': 5, 'AD2': 6,
       'SR': 8, 'RR': 9, 'SL': 10, 'RL': 11, 'RL8': 15}
SRC = {'M0': 0, 'M1': 1, 'M2': 2, 'M3': 3, 'MC0': 4, 'MC1': 5, 'MC2': 6, 'MC3': 7}
D1SRC = dict(SRC, ALL=9, ALH=10)
D1DST = {'MC0': 0, 'MC1': 1, 'MC2': 2, 'MC3': 3, 'RX': 4, 'PL': 5, 'RA0': 6, 'WA0': 7,
         'LOP': 10, 'TOP': 11, 'CT0': 12, 'CT1': 13, 'CT2': 14, 'CT3': 15}
MVIDST = {'MC0': 0, 'MC1': 1, 'MC2': 2, 'MC3': 3, 'RX': 4, 'PL': 5, 'RA0': 6, 'WA0': 7,
          'LOP': 10, 'PC': 12}
COND = {'NZ': 0x01, 'NS': 0x02, 'NZS': 0x03, 'NC': 0x04, 'NT0': 0x08,
        'Z': 0x21, 'S': 0x22, 'ZS': 0x23, 'C': 0x24, 'T0': 0x28}
DMARAM = {'MC0': 0, 'MC1': 1, 'MC2': 2, 'MC3': 3, 'PRG': 4}


class AsmError(Exception):
    pass


def _imm(text, bits, labels=None):
    text = text.strip().lstrip('#')
    if labels is not None and text in labels:
        value = labels[text]
    else:
        value = int(text, 0)
    lo, hi = -(1 << (bits - 1)), (1 << bits) - 1
    if not lo <= value <= hi:
        raise AsmError('immediate %s out of %d-bit range' % (text, bits))
    return value & ((1 << bits) - 1)


def _operation(fields):
    alu = x = y = d1 = None
    for f in fields:
        u = ' '.join(f.upper().split())
        if u in ALU:
            if alu is not None:
                raise AsmError('two ALU fields')
            alu = ALU[u]
            continue
        m = re.fullmatch(r'MOV (\S+),(\S+)', u)
        if u == 'CLR A':
            y = (y or (0, 0))
            y = (y[0] | 0b001, y[1])
            continue
        if not m:
            raise AsmError('unknown field %r' % f)
        s, d = m.groups()
        if d == 'X' and s in SRC:
            x = (x or (0, 0)); x = (x[0] | 0b100, SRC[s])
        elif d == 'P' and s == 'MUL':
            x = (x or (0, 0)); x = (x[0] | 0b010, x[1])
        elif d == 'P' and s in SRC:
            x = (x or (0, 0)); x = (x[0] | 0b011, SRC[s])
        elif d == 'Y' and s in SRC:
            y = (y or (0, 0)); y = (y[0] | 0b100, SRC[s])
        elif d == 'A' and s == 'ALU':
            y = (y or (0, 0)); y = (y[0] | 0b010, y[1])
        elif d == 'A' and s in SRC:
            y = (y or (0, 0)); y = (y[0] | 0b011, SRC[s])
        elif d in D1DST:
            if d1 is not None:
                raise AsmError('two D1-bus fields')
            if s.startswith('#'):
                d1 = (0b01, D1DST[d], _imm(s, 8))
            elif s in D1SRC:
                d1 = (0b11, D1DST[d], D1SRC[s])
            else:
                raise AsmError('bad D1 source %r' % s)
        else:
            raise AsmError('unknown field %r' % f)
    w = (alu or 0) << 26
    if x:
        w |= x[0] << 23 | x[1] << 20
    if y:
        w |= y[0] << 17 | y[1] << 14
    if d1:
        w |= d1[0] << 12 | d1[1] << 8 | d1[2]
    return w


def _special(line, labels):
    u = ' '.join(line.upper().split())
    if u == 'END':
        return 0xF0000000
    if u == 'ENDI':
        return 0xF8000000
    if u == 'LPS':
        return 0xE8000000
    if u == 'BTM':
        return 0xE0000000
    m = re.fullmatch(r'MVI (\S+),(\S+)', u)
    if m:
        if m.group(2) not in MVIDST:
            raise AsmError('bad MVI destination')
        return 0x80000000 | MVIDST[m.group(2)] << 26 | _imm(m.group(1), 25, labels)
    m = re.fullmatch(r'JMP (?:(\S+),)?(\S+)', u)
    if m:
        w = 0xD0000000 | _imm(m.group(2), 8, labels)
        if m.group(1):
            w |= 1 << 25 | COND[m.group(1)] << 19
        return w
    m = re.fullmatch(r'(DMAH?) (\S+),(\S+),(\S+)', u)
    if m:
        hold = m.group(1) == 'DMAH'
        a, b, n = m.group(2), m.group(3), m.group(4)
        w = 0xC0000000 | (1 << 14 if hold else 0) | _imm(n, 8)
        if a == 'D0':      # D0 -> DSP: stride field 2 = +4 bytes
            w |= DMARAM[b] << 8 | 2 << 15
        elif b == 'D0':    # DSP -> D0
            w |= 1 << 12 | DMARAM[a] << 8 | 2 << 15
        else:
            raise AsmError('DMA needs D0 on one side')
        return w
    return None


def assemble(text):
    lines, labels = [], {}
    for raw in text.splitlines():
        line = raw.split(';', 1)[0].strip()
        while ':' in line.split(' ', 1)[0]:
            name, line = line.split(':', 1)
            labels[name.strip().upper()] = len(lines)
            line = line.strip()
        if line:
            lines.append(line)
    words = []
    for n, line in enumerate(lines):
        try:
            w = _special(line, labels)
            if w is None:
                w = _operation([f for f in line.split('|') if f.strip()])
        except (AsmError, KeyError, ValueError) as e:
            raise AsmError('instruction %d %r: %s' % (n, line, e))
        words.append(w)
    if len(words) > 256:
        raise AsmError('program has %d words; program RAM holds 256' % len(words))
    return words, labels


def main():
    if len(sys.argv) != 4:
        raise SystemExit(__doc__)
    words, _ = assemble(open(sys.argv[1]).read())
    sym = sys.argv[3]
    body = ',\n'.join('    0x%08Xu' % w for w in words)
    out = ('/* Generated by saturn/tools/sh2z_dspasm.py from %s; regenerate, do not edit. */\n'
           '#define %s_WORDS %du\n'
           'static const unsigned long %s[%s_WORDS] = {\n%s\n};\n'
           % (sys.argv[1].replace('\\', '/').split('/')[-1], sym.upper(), len(words), sym, sym.upper(), body))
    open(sys.argv[2], 'w', newline='\n').write(out)


if __name__ == '__main__':
    main()
