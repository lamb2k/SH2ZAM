#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 SH2ZAM Contributors.
"""Per-symbol identity of two sets of SH-2 objects (big-endian ELF32 relocatable
files with the same names), for changes that must not change any function's
code: base and new are directories of objects compiled with the same flags.

    python3 saturn/tools/symbol_identity.py BASE_DIR NEW_DIR

For every defined global symbol, and every local function or object, of every
object, one line with the strongest of:
  strict  same size, same bytes, same relocations (targets resolved to the
          symbol and offset they reach, in-place addends included), same
          address modulo 4;
  reloc   the same, except the words a relocation fills (which hold a section
          offset that moves when sections are split) may differ;
  code    the code reachable from the entry, followed through branches, BSR
          calls, delay slots, PC-relative literals and MOVA tables, is the same
          instruction for instruction at the same address modulo 4, every
          branch reaching the same target instruction and every literal the
          same value or relocation target; only PC-relative displacement
          fields may differ.
It exits 1 if any symbol is none of these, or is missing on one side.
No dependencies beyond Python 3. sections-1 used it on main against the branch,
under the CI flags and under -ffunction-sections -fdata-sections."""
import hashlib, struct, sys
from pathlib import Path


class Elf:
    def __init__(self, path):
        d = path.read_bytes()
        self.d = d
        assert d[:4] == b'\x7fELF' and d[4] == 1 and d[5] == 2, path
        (shoff,) = struct.unpack_from('>I', d, 0x20)
        shentsize, shnum, shstrndx = struct.unpack_from('>HHH', d, 0x2e)
        self.sh = []
        for i in range(shnum):
            name, typ, flags, addr, off, size, link, info, align, entsize = struct.unpack_from('>10I', d, shoff + i * shentsize)
            self.sh.append(dict(name=name, type=typ, flags=flags, off=off, size=size, link=link, info=info, align=align, entsize=entsize))
        strtab = self.sh[shstrndx]
        for s in self.sh:
            s['name'] = self.cstr(strtab['off'] + s['name'])
        self.syms = []
        for i, s in enumerate(self.sh):
            if s['type'] == 2:  # SYMTAB
                st = self.sh[s['link']]
                for j in range(s['size'] // 16):
                    nm, val, sz, info, oth, shndx = struct.unpack_from('>IIIBBH', d, s['off'] + 16 * j)
                    self.syms.append(dict(name=self.cstr(st['off'] + nm), value=val, size=sz,
                                          bind=info >> 4, type=info & 15, shndx=shndx))
        self.rel = {}  # section index -> {offset: (type, symidx, addend)}
        for s in self.sh:
            if s['type'] == 4:  # RELA
                m = self.rel.setdefault(s['info'], {})
                for j in range(s['size'] // 12):
                    off, info, add = struct.unpack_from('>IIi', d, s['off'] + 12 * j)
                    m[off] = (info & 255, info >> 8, add)

    def cstr(self, o):
        return self.d[o:self.d.index(b'\0', o)].decode()

    def data(self, shndx):
        s = self.sh[shndx]
        return b'\0' * s['size'] if s['type'] == 8 else self.d[s['off']:s['off'] + s['size']]

    def defined(self):
        """symbol name -> symbol, for globals and local functions/objects."""
        out = {}
        for s in self.syms:
            if s['shndx'] == 0 or s['shndx'] >= 0xff00 or not s['name']:
                continue
            if s['bind'] in (1, 2) or s['type'] in (1, 2):
                out[s['name']] = s
        return out

    def owner(self, shndx, off):
        """The named symbol whose range holds (section, offset), as name+delta."""
        best = None
        for s in self.syms:
            if s['shndx'] == shndx and s['name'] and s['type'] in (1, 2) and s['value'] <= off < s['value'] + max(s['size'], 1):
                if best is None or s['value'] > best['value'] or (s['value'] == best['value'] and s['bind'] > best['bind']):
                    best = s
        return best

    def reloc_target(self, shndx, off):
        r = self.rel.get(shndx, {}).get(off)
        if r is None:
            return None
        typ, si, add = r
        if typ in (1, 2):  # R_SH_DIR32, R_SH_REL32: gas keeps the addend in place
            (inplace,) = struct.unpack_from('>i', self.data(shndx), off)
            add += inplace
        s = self.syms[si]
        if s['type'] == 3 or not s['name']:  # section symbol: resolve to an owner
            tsec, toff = s['shndx'], s['value'] + add
            o = self.owner(tsec, toff)
            if o:
                return (typ, 'sym', o['name'], toff - o['value'])
            return (typ, 'anon', '.' + self.sh[tsec]['name'].lstrip('.').split('.')[0],
                    hashlib.sha1(self.data(tsec)[toff:toff + 16]).hexdigest()[:12], tsec, toff)
        return (typ, 'sym', s['name'], add)


def s16(v):
    return v - 0x10000 if v & 0x8000 else v


def flow(e, shndx, entry, limit=None):
    """Canonical form of the code reachable from entry in section shndx."""
    code = e.data(shndx)
    labels, work, seen, out = {}, [], set(), []

    def label(a):
        if a not in labels:
            labels[a] = 'L%d' % len(labels)
            work.append(a)
        return labels[a]

    def rtok(r):
        if r is None:
            return None
        if r[1] == 'anon':
            if r[4] == shndx:  # pointer into this code section: follow it
                return ('ptr', r[0], label(r[5]))
            return ('anon', r[0], r[2], r[3])
        return r[:4]

    def lit(a, n):
        r = e.reloc_target(shndx, a)
        if r is not None:
            return rtok(r)
        return code[a:a + n].hex()

    label(entry)
    while work:
        a = work.pop()
        if a in seen:
            continue
        out.append(('block', labels[a], a & 3))
        delay_end = None
        while True:
            if a in seen:
                out.append(('goto', label(a)))
                break
            seen.add(a)
            if a + 2 > len(code):
                out.append(('end-of-section',))
                break
            w = (code[a] << 8) | code[a + 1]
            hi = w >> 12
            tok = None
            stop = False
            if hi in (0xA, 0xB):  # BRA / BSR
                disp = w & 0xFFF
                disp = disp - 0x1000 if disp & 0x800 else disp
                t = a + 4 + 2 * disp
                tok = ('bra' if hi == 0xA else 'bsr', label(t))
                if hi == 0xA:
                    delay_end = a + 4
            elif (w >> 8) in (0x89, 0x8B, 0x8D, 0x8F):  # BT BF BT/S BF/S
                disp = w & 0xFF
                disp = disp - 0x100 if disp & 0x80 else disp
                tok = ('bcc', w >> 8, label(a + 4 + 2 * disp))
            elif hi == 0x9:  # MOV.W @(disp,PC),Rn
                la = a + 4 + 2 * (w & 0xFF)
                tok = ('movw-pc', (w >> 8) & 15, lit(la, 2))
            elif hi == 0xD:  # MOV.L @(disp,PC),Rn
                la = (a & ~3) + 4 + 4 * (w & 0xFF)
                tok = ('movl-pc', (w >> 8) & 15, lit(la, 4))
            elif (w >> 8) == 0xC7:  # MOVA
                la = (a & ~3) + 4 + 4 * (w & 0xFF)
                tok = ('mova', tuple(lit(la + 4 * k, 4) for k in range(8)))
            elif w in (0x000B, 0x002B) or (w & 0xF0FF) in (0x402B, 0x0023):  # RTS RTE JMP BRAF
                tok = ('raw', '%04x' % w)
                delay_end = a + 4
            else:
                tok = ('raw', '%04x' % w)
            out.append(tok)
            a += 2
            if delay_end is not None and a == delay_end:
                break
        # fall-through of a conditional branch or call is covered by the loop
    return out


def main():
    base, lane = Path(sys.argv[1]), Path(sys.argv[2])
    bad = 0
    counts = {'strict': 0, 'reloc': 0, 'code': 0}
    for bo in sorted(base.glob('*.o')):
        lo = lane / bo.name
        b, l = Elf(bo), Elf(lo)
        bd, ld = b.defined(), l.defined()
        for name in sorted(set(bd) | set(ld)):
            if name not in bd or name not in ld:
                print('MISSING', bo.name, name, 'base' if name in bd else 'lane')
                bad += 1
                continue
            bs, ls = bd[name], ld[name]
            bsec, lsec = b.sh[bs['shndx']], l.sh[ls['shndx']]
            bbytes = b.data(bs['shndx'])[bs['value']:bs['value'] + bs['size']]
            lbytes = l.data(ls['shndx'])[ls['value']:ls['value'] + ls['size']]
            brel = [(o - bs['value'], b.reloc_target(bs['shndx'], o)) for o in sorted(b.rel.get(bs['shndx'], {})) if bs['value'] <= o < bs['value'] + bs['size']]
            lrel = [(o - ls['value'], l.reloc_target(ls['shndx'], o)) for o in sorted(l.rel.get(ls['shndx'], {})) if ls['value'] <= o < ls['value'] + ls['size']]
            norm = lambda rs: [(o, r[:4]) for o, r in rs]  # anon targets: kind and content hash, not offset
            where = '%s -> %s' % (bsec['name'], lsec['name'])
            same_align = (bs['value'] & 3) == (ls['value'] & 3)
            if bbytes == lbytes and norm(brel) == norm(lrel) and same_align:
                counts['strict'] += 1
                print('strict  %-28s %-44s %6d  %s' % (bo.name, name, bs['size'], where))
                continue

            def masked(bytes_, rels):
                m = bytearray(bytes_)
                for o, r in rels:
                    if r and r[0] in (1, 2):
                        m[o:o + 4] = b'\0\0\0\0'
                return bytes(m)
            if (len(bbytes) == len(lbytes) and same_align and norm(brel) == norm(lrel)
                    and masked(bbytes, brel) == masked(lbytes, lrel)):
                counts['reloc'] += 1
                print('reloc   %-28s %-44s %6d  %s  (bytes identical outside %d relocated words; same targets)' % (bo.name, name, bs['size'], where, len(brel)))
                continue
            if bs['type'] == 2 and bsec['flags'] & 4:
                fb, fl = flow(b, bs['shndx'], bs['value']), flow(l, ls['shndx'], ls['value'])
                if fb == fl:
                    counts['code'] += 1
                    ninsn = sum(1 for t in fb if t[0] not in ('block', 'goto'))
                    print('code    %-28s %-44s %6d->%-6d %d insns reachable  %s' % (bo.name, name, bs['size'], ls['size'], ninsn, where))
                    continue
                for i, (x, y) in enumerate(zip(fb, fl)):
                    if x != y:
                        print('   first difference at token', i, x, y)
                        break
                else:
                    print('   lengths', len(fb), len(fl))
            print('DIFF    %-28s %-44s %6d->%-6d  %s' % (bo.name, name, bs['size'], ls['size'], where))
            bad += 1
    print('summary: %d strict, %d reloc-identical, %d code-identical, %d different or missing' % (counts['strict'], counts['reloc'], counts['code'], bad))
    sys.exit(1 if bad else 0)


main()
