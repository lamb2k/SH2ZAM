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
     D1:    MOV #imm8,[d]   MOV [s],[d]     ([s] may also be ALL or ALH;
            #imm8 may be a label)
  MVI #imm,[d]          25-bit signed immediate ([d]: MC0-3 RX PL RA0 WA0 LOP PC)
  MVI #imm,[d],cond     19-bit signed immediate, loaded only when cond holds
  DMA D0,[MCn|PRG],n    DMA [MCn],D0,n   (DMAH: hold the D0 address)
                        n is #imm8, or Mn / MCn: the count is read from data RAM
  JMP label | JMP cond,label   (cond: Z NZ S NS C NC T0 NT0 ZS NZS)
  LPS   BTM   END   ENDI
  ([s]: M0-3 read without, MC0-3 with post-increment of CT0-3)

Usage:
  sh2z_dspasm.py IN.dsp OUT.inc          write the words as a C initializer body
  sh2z_dspasm.py --check IN.dsp OUT.inc  exit 1 unless OUT.inc is up to date
  sh2z_dspasm.py --self-test [--control] encodings against hand-derived words;
                                         --control alters one expectation and
                                         must make the self-test fail
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


def _imm(text, bits, labels=None, signed=True):
    text = text.strip().lstrip('#')
    if labels is not None and text in labels:
        value = labels[text]
    else:
        value = int(text, 0)
    lo = -(1 << (bits - 1)) if signed else 0
    hi = (1 << bits) - 1
    if not lo <= value <= hi:
        raise AsmError('immediate %s out of %d-bit range' % (text, bits))
    return value & ((1 << bits) - 1)


def _operation(fields, labels):
    alu = d1 = None
    xop = yop = 0
    xsrc = ysrc = None

    def source(old, new):
        if old is not None and old != new:
            raise AsmError('one bus, two sources')
        return new

    for f in fields:
        u = ' '.join(f.upper().split())
        if u in ALU:
            if alu is not None:
                raise AsmError('two ALU fields')
            alu = ALU[u]
            continue
        if u == 'CLR A':
            if yop & 0b011:
                raise AsmError('Y-bus: two A operations')
            yop |= 0b001
            continue
        m = re.fullmatch(r'MOV (\S+),(\S+)', u)
        if not m:
            raise AsmError('unknown field %r' % f)
        s, d = m.groups()
        if d == 'X' and s in SRC:
            if xop & 0b100:
                raise AsmError('two X loads')
            xop |= 0b100
            xsrc = source(xsrc, SRC[s])
        elif d == 'P' and (s == 'MUL' or s in SRC):
            if xop & 0b011:
                raise AsmError('X-bus: two P operations')
            if s == 'MUL':
                xop |= 0b010
            else:
                xop |= 0b011
                xsrc = source(xsrc, SRC[s])
        elif d == 'Y' and s in SRC:
            if yop & 0b100:
                raise AsmError('two Y loads')
            yop |= 0b100
            ysrc = source(ysrc, SRC[s])
        elif d == 'A' and (s == 'ALU' or s in SRC):
            if yop & 0b011:
                raise AsmError('Y-bus: two A operations')
            if s == 'ALU':
                yop |= 0b010
            else:
                yop |= 0b011
                ysrc = source(ysrc, SRC[s])
        elif d in D1DST:
            if d1 is not None:
                raise AsmError('two D1-bus fields')
            if s.startswith('#'):
                d1 = (0b01, D1DST[d], _imm(s, 8, labels))
            elif s in D1SRC:
                d1 = (0b11, D1DST[d], D1SRC[s])
            else:
                raise AsmError('bad D1 source %r' % s)
        else:
            raise AsmError('unknown field %r' % f)
    w = (alu or 0) << 26 | xop << 23 | (xsrc or 0) << 20 | yop << 17 | (ysrc or 0) << 14
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
    m = re.fullmatch(r'MVI (\S+?),(\S+?)(?:,(\S+))?', u)
    if m:
        if m.group(2) not in MVIDST:
            raise AsmError('bad MVI destination')
        w = 0x80000000 | MVIDST[m.group(2)] << 26
        if m.group(3):
            if m.group(3) not in COND:
                raise AsmError('bad MVI condition')
            return w | 1 << 25 | COND[m.group(3)] << 19 | _imm(m.group(1), 19, labels)
        return w | _imm(m.group(1), 25, labels)
    m = re.fullmatch(r'JMP (?:(\S+),)?(\S+)', u)
    if m:
        w = 0xD0000000 | _imm(m.group(2), 8, labels, signed=False)
        if m.group(1):
            if m.group(1) not in COND:
                raise AsmError('bad jump condition')
            w |= 1 << 25 | COND[m.group(1)] << 19
        return w
    m = re.fullmatch(r'(DMAH?) (\S+),(\S+),(\S+)', u)
    if m:
        hold = m.group(1) == 'DMAH'
        a, b, n = m.group(2), m.group(3), m.group(4)
        w = 0xC0000000 | (1 << 14 if hold else 0)
        if n.startswith('#'):
            w |= _imm(n, 8, labels, signed=False)
        elif n in SRC:              # count from data RAM; MCn also increments CTn
            w |= 1 << 13 | SRC[n]
        else:
            raise AsmError('bad DMA count %r' % n)
        if a == 'D0' and b in DMARAM:      # D0 -> DSP: add mode 2 = +4 bytes
            w |= DMARAM[b] << 8 | 2 << 15
        elif b == 'D0' and a in DMARAM and a != 'PRG':   # DSP -> D0
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
            name = name.strip().upper()
            if name in labels:
                raise AsmError('label %s defined twice' % name)
            labels[name] = len(lines)
            line = line.strip()
        if line:
            lines.append(line)
    words = []
    for n, line in enumerate(lines):
        try:
            w = _special(line, labels)
            if w is None:
                w = _operation([f for f in line.split('|') if f.strip()], labels)
        except (AsmError, KeyError, ValueError) as e:
            raise AsmError('instruction %d %r: %s' % (n, line, e))
        words.append(w)
    if len(words) > 256:
        raise AsmError('program has %d words; program RAM holds 256' % len(words))
    return words, labels


def render(words, source_name):
    body = ''.join('    UINT32_C(0x%08X),\n' % w for w in words)
    return ('/* Generated by saturn/tools/sh2z_dspasm.py from %s: %d words.\n'
            ' * Regenerate, do not edit.\n'
            ' *\n'
            ' * Copyright (c) 2026 SH2ZAM Contributors. MIT License.\n'
            ' * Inspired by SH4ZAM by Falco Girgis and SH4ZAM Contributors.\n'
            ' */\n%s'
            % (source_name, len(words), body))


# Hand-derived encodings, from the SCU DSP field layout: operation words
# ALU 29-26, X op 25-23 / src 22-20, Y op 19-17 / src 16-14, D1 op 13-12 /
# dst 11-8 / imm or src 7-0; MVI 31-30 = 10, dst 29-26, cond flag 25;
# DMA 31-28 = 1100, add 17-15, hold 14, count-from-RAM 13, to D0 12,
# RAM 10-8, count 7-0; JMP 31-28 = 1101, cond flag 25, cond 24-19, target
# 7-0; loop 31-28 = 1110 (LPS bit 27); end 31-28 = 1111 (ENDI bit 27).
KNOWN = [
    ('NOP', 0x00000000), ('END', 0xF0000000), ('ENDI', 0xF8000000),
    ('BTM', 0xE0000000), ('LPS', 0xE8000000),
    ('MOV MC0,X', 0x02400000), ('MOV MC1,Y', 0x00094000), ('MOV MUL,P', 0x01000000),
    ('CLR A', 0x00020000), ('MOV ALU,A', 0x00040000), ('MOV M3,A', 0x0006C000),
    ('MOV M2,P', 0x01A00000), ('ADD', 0x10000000), ('SUB', 0x14000000), ('AD2', 0x18000000),
    ('RL8', 0x3C000000), ('MOV ALL,MC2', 0x00003209), ('MOV ALH,MC1', 0x0000310A),
    ('MOV #1,CT3', 0x00001F01), ('MOV #-1,RX', 0x000014FF), ('MOV MC0,MC3', 0x00003304),
    ('MOV M3,RA0', 0x00003603), ('MOV #8,TOP', 0x00001B08),
    ('MOV MUL,P | MOV MC1,X | CLR A | MOV MC0,Y | MOV ALL,MC2', 0x035B3209),
    ('MOV M3,A | MOV #15,PL', 0x0006D50F),
    ('AD2 | MOV MUL,P | MOV ALU,A | MOV ALH,MC2', 0x1904320A),
    ('MVI #19,LOP', 0xA8000013), ('MVI #1,MC3', 0x8C000001), ('MVI #-2,RX', 0x91FFFFFE),
    ('MVI #5,PC,NZ', 0xB2080005),
    ('DMA D0,MC0,#16', 0xC0010010), ('DMA D0,MC1,#60', 0xC001013C),
    ('DMA MC2,D0,#60', 0xC001123C), ('DMAH D0,PRG,#3', 0xC0014403),
    ('DMA D0,MC1,MC3', 0xC0012107),
    ('JMP 4', 0xD0000004), ('JMP T0,4', 0xD3400004), ('JMP NZ,5', 0xD2080005),
]
BAD = ['ADD | SUB', 'MOV MUL,P | MOV M1,P', 'MOV M0,X | MOV M1,P', 'CLR A | MOV ALU,A',
       'MOV #300,CT0', 'MOV #1,CT0 | MOV #2,CT1', 'MVI #1,CT0', 'DMA MC0,MC1,#1',
       'JMP Q,1', 'FOO']


def self_test(control):
    known = list(KNOWN)
    if control:
        known[5] = (known[5][0], known[5][1] ^ 1)
    fails = 0
    for text, want in known:
        got = assemble(text)[0][0]
        if got != want:
            print('encoding %r: got %08X want %08X' % (text, got, want))
            fails += 1
    for text in BAD:
        try:
            assemble(text)
            print('accepted bad input %r' % text)
            fails += 1
        except AsmError:
            pass
    words, labels = assemble('A: NOP\nB: JMP A\nMOV #B,TOP\n')
    if words != [0, 0xD0000000, 0x00001B01] or labels != {'A': 0, 'B': 1}:
        print('labels: %r %r' % (words, labels))
        fails += 1
    print('%d encodings, %d rejects, labels: %s' % (len(known), len(BAD), 'FAIL' if fails else 'pass'))
    return 1 if fails else 0


def main():
    args = sys.argv[1:]
    if args[:1] == ['--self-test']:
        raise SystemExit(self_test(args[1:] == ['--control']))
    check = args[:1] == ['--check']
    if check:
        args = args[1:]
    if len(args) != 2:
        raise SystemExit(__doc__)
    words, _ = assemble(open(args[0]).read())
    text = render(words, args[0].replace('\\', '/').split('/')[-1])
    if check:
        try:
            current = open(args[1], newline='').read()
        except OSError:
            current = None
        if current != text:
            print('%s is stale: regenerate it from %s' % (args[1], args[0]))
            raise SystemExit(1)
        print('%s matches %s (%d words)' % (args[1], args[0], len(words)))
        return
    open(args[1], 'w', newline='\n').write(text)


if __name__ == '__main__':
    main()
