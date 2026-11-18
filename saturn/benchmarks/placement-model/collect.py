#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
"""Package the placement model's logs (MODEL_PLACEMENT_LOG of the view, shade
and vlc captures) into one results file. MODELLED tier only: region costs come
from hardware notes of a Saturn port, not from this project's measurements.

For every kernel the CPU-only figure is recomputed from the log with the
statistic its own capture publishes, and must equal that capture's published
value exactly; the placed figures use the same statistic over placed cycles
(CPU-only cycles plus the stalls the placement adds)."""
import argparse, hashlib, json, sys
from pathlib import Path

parser = argparse.ArgumentParser()
for suite in ('view', 'shade', 'vlc'):
    parser.add_argument(f'--{suite}-log', type=Path, required=True)
    parser.add_argument(f'--{suite}-results', type=Path, required=True)
parser.add_argument('--committed-root', type=Path, help='repository root: also compare CPU-only values with the committed captures')
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--markdown', type=Path, help='write the README tables here')
parser.add_argument('--evidence-url', required=True)
a = parser.parse_args()
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

FIELDS = ['stall', 'fillCode', 'fillTables', 'fillInput', 'fillOutput', 'fillStack', 'through', 'writeWait']
MODES = ('warm', 'cold')


def load_log(path):
    lines = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
    head = lines[0]
    assert head['placementModel'] == 1 and head['fields'] == FIELDS and head['selfTest'] == 'passed'
    declares = {l['declare']: l['symbols'] for l in lines if 'declare' in l}
    calls = {}
    for l in lines:
        if 'tag' not in l:
            continue
        assert len(l['placed']) == len(head['placements'])
        for per in l['placed']:
            for run in per:
                assert len(run) == len(FIELDS) and sum(run[1:]) == run[0], 'stall components must add up'
        calls.setdefault(l['tag'], []).append(l)
    return head, declares, calls


suites = {}
for s in ('view', 'shade', 'vlc'):
    head, declares, calls = load_log(getattr(a, f'{s}_log'))
    suites[s] = dict(head=head, declares=declares, calls=calls,
                     results=json.loads(getattr(a, f'{s}_results').read_text()))
names = [p['name'] for p in suites['view']['head']['placements']]
for s in suites.values():
    assert [p['name'] for p in s['head']['placements']] == names
    assert s['head']['regions'] == suites['view']['head']['regions']


def cycles(call, p=None, mode=0):
    """CPU-only cycles (p None) or placed cycles of one call."""
    return call['cycles'] if p is None else call['cycles'] + call['placed'][p][mode][0]


def series(suite, tag, p=None, mode=0):
    c = suites[suite]['calls'].get(tag)
    assert c, f'no calls logged for {tag}'
    return [cycles(x, p, mode) for x in c]


# Statistics, each exactly as its capture prints it.
def view_median(values, per, decimals):
    v = sorted(values)[len(values) // 2]
    return int(v) if decimals is None else round(float('%.*f' % (decimals, v / per)), decimals)


def summarize_median(values):
    v = sorted(values); n = len(v)
    return float('%.1f' % (v[n // 2] if n & 1 else (v[n // 2 - 1] + v[n // 2]) / 2.0))


def mean2(values):
    return float('%.2f' % (sum(values) / len(values)))


def shade_slope(small, big, n):
    return round((mean2(big) - mean2(small)) / (n - 1), 2)


def vlc_slope(one, many, n):
    return float('%.2f' % ((sum(many) / len(many) - sum(one) / len(one)) / (n - 1)))


def dig(obj, path):
    for k in path:
        obj = obj[k]
    return obj


def make(spec):
    suite = spec['suite']; compute = spec['compute']
    cpu = compute(None, 0)
    published = dig(suites[suite]['results'], spec['path'])
    assert cpu == published, f"{spec['id']}: CPU-only {cpu} recomputed from the placement log differs from the published {published}"
    out = dict(id=spec['id'], function=spec['function'], role=spec['role'], suite=suite, fixture=spec['fixture'],
               unit=spec['unit'], statistic=spec['statistic'], cpuOnly=cpu,
               published=dict(file=f'saturn/benchmarks/{suite}-model/results.json', path='.'.join(spec['path']), value=published))
    if a.committed_root:
        committed = json.loads((a.committed_root / out['published']['file']).read_text())
        out['published']['matchesCommitted'] = dig(committed, spec['path']) == cpu
    out['placed'] = {n: {m: compute(i, k) for k, m in enumerate(MODES)} for i, n in enumerate(names)}
    if 'median' in spec['statistic']:
        # Stalls are never negative, so a median of placed cycles is never below the CPU-only median.
        assert all(v['warm'] >= cpu and v['cold'] >= cpu for v in out['placed'].values()), spec['id']
    # Where the stall goes: percent of all stall cycles over the fixture's calls.
    share = {}
    for i, n in enumerate(names):
        share[n] = {}
        for k, m in enumerate(MODES):
            tot = [0] * len(FIELDS)
            for tag in spec['tags']:
                for c in suites[suite]['calls'][tag]:
                    tot = [x + y for x, y in zip(tot, c['placed'][i][k])]
            share[n][m] = {f: (round(100.0 * tot[j] / tot[0], 1) if tot[0] else 0.0) for j, f in enumerate(FIELDS) if j}
    out['stallShare'] = share
    out['declared'] = suites[suite]['declares'][spec['tags'][0]]
    return out


specs = []
VIEW_ROWS = [('batch', 'sh2z_view_batch_center_divu', 512, 'cycles per vertex', '512 vertices inside the view volume, 64 cameras'),
             ('project', 'sh2z_view_project_center_divu', 512, 'cycles per point', '512 view-space points, 64 cameras'),
             ('edgeSteps', 'sh2z_view_edge_steps_center_divu', 64, 'cycles per point', 'one edge, 64 points, 64 cameras'),
             ('clipNear', 'sh2z_view_clip_near_center_divu', 1, 'cycles per quad', 'one quad, two corners behind the near plane, 64 cameras'),
             ('guardInside', 'sh2z_view_clip_guard_center_divu', 1, 'cycles per polygon', 'one quad inside the band, attrs 2, 64 cameras'),
             ('guardSide', 'sh2z_view_clip_guard_center_divu', 1, 'cycles per polygon', 'one quad crossing a side plane, attrs 2, 64 cameras'),
             ('guardNearSide', 'sh2z_view_clip_guard_center_divu', 1, 'cycles per polygon', 'one quad crossing the near and a side plane, attrs 2, 64 cameras')]
for row, fn, per, unit, fixture in VIEW_ROWS:
    for form, suffix, role in (('centerDivu', '', 'library (divider)'), ('centerPortable', '_portable', 'library (portable)')):
        tag = f'view.center.{row}.{form}'
        name = fn if form == 'centerDivu' else fn.replace('_divu', '')
        specs.append(dict(id=f'center.{row}{suffix}', function=name, role=role, suite='view', fixture=fixture, unit=unit,
                          statistic=f'median over 64 trials of call cycles / {per}', tags=[tag],
                          path=['center', 'timing', row, 'median', form],
                          compute=lambda p, m, tag=tag, per=per: view_median(series('view', tag, p, m), per, 2)))
for row, fixture in (('inside', 'one quad inside the band, attrs 2'), ('side', 'one quad crossing a side plane, attrs 2'),
                     ('nearSide', 'one quad crossing the near and a side plane, attrs 2'),
                     ('roadNear', 'one road quad from behind the camera, attrs 0')):
    for form, suffix, role in (('centerDivu', '', 'library (divider)'), ('centerPortable', '_portable', 'library (portable)')):
        tag = f'view.srcGuard.{row}.{form}'
        specs.append(dict(id=f'srcGuard.{row}{suffix}', function='sh2z_view_clip_guard_src_center' + ('_divu' if not suffix else ''),
                          role=role, suite='view', fixture=fixture + ', 64 cameras', unit='cycles per polygon',
                          statistic='median over 64 trials of call cycles', tags=[tag],
                          path=['center', 'srcGuard', 'timing', row, 'median', form],
                          compute=lambda p, m, tag=tag: view_median(series('view', tag, p, m), 1, None)))
for cfg, fixture in (('pixel1', 'pixel mode, 1 px tolerance'), ('pixel0.5', 'pixel mode, 0.5 px'), ('pixel2', 'pixel mode, 2 px'),
                     ('pixel1Aligned', 'pixel mode, 1 px, end masks'), ('rows1', 'row mode, 1 px')):
    tag = f'view.merge.{cfg}'
    specs.append(dict(id=f'merge.{cfg}', function='sh2z_view_merge_runs', role='library', suite='view',
                      fixture=f'64-boundary road strip, runs 1/2/4, {fixture}, 64 strips', unit='cycles per boundary',
                      statistic='median over 64 strips of call cycles / 64', tags=[tag],
                      path=['merge', 'timing', 'configs', cfg, 'medianCyclesPerBoundary'],
                      compute=lambda p, m, tag=tag: view_median(series('view', tag, p, m), 64, 2)))
SHADE = [('light_1', 'light_256', 256, 'normal', [('fast_light_normals', 'sh2z_shade_light_normals', 'library'), ('fast_light', 'sh2z_shade_light_prepare + _normals', 'library'), ('plain_light', 'port C', 'port C')]),
         ('gouraud_1', 'gouraud_448', 448, 'pair', [('fast_gouraud', 'sh2z_shade_gouraud', 'library'), ('plain_gouraud', 'port C', 'port C')]),
         ('cull_1', 'cull_256', 256, 'face', [('fast_cull', 'sh2z_shade_cull', 'library'), ('plain_cull', 'port C', 'port C')]),
         ('fog_1', 'fog_1024_uniform', 1024, 'depth', [('fast_fog', 'sh2z_shade_fog_lookup', 'library'), ('plain_fog', 'port C', 'port C')]),
         ('fade_1', 'fade_1024', 1024, 'fade', [('fast_offset', 'sh2z_shade_fade_offset', 'library'), ('plain_offset', 'port C', 'port C'),
                                               ('fast_mix', 'sh2z_shade_fade_mix', 'library'), ('plain_mix', 'port C', 'port C')])]
for small, big, n, elem, entries in SHADE:
    for entry, fn, role in entries:
        ts, tb = f'shade.{small}.{entry}', f'shade.{big}.{entry}'
        specs.append(dict(id=f'shade.{entry}', function=fn, role=role, suite='shade',
                          fixture=f'{big}: {n} {elem}s against the 1-{elem} call, 64 corpora', unit=f'cycles per {elem}',
                          statistic=f'(mean of the {n}-{elem} calls - mean of the 1-{elem} calls) / {n - 1}, means at 2 decimals',
                          tags=[tb], path=['cyclesPerElement', big, entry],
                          compute=lambda p, m, ts=ts, tb=tb, n=n: shade_slope(series('shade', ts, p, m), series('shade', tb, p, m), n)))
for entry, fn, role in (('fast_fog_build', 'sh2z_shade_fog_build', 'library'), ('plain_fog_build', 'port C', 'port C')):
    tag = f'shade.fog_build.{entry}'
    specs.append(dict(id=f'shade.{entry}', function=fn, role=role, suite='shade', fixture='one 256-entry fog table, 64 corpora',
                      unit='cycles per table', statistic='median over 64 corpora', tags=[tag],
                      path=['rows', 'fog_build', entry, 'medianCycles'],
                      compute=lambda p, m, tag=tag: summarize_median(series('shade', tag, p, m))))
for entry, fn, role in (('fast_frame', 'light + cull + gouraud kernels', 'library'), ('fast_frame_baked', 'kernels, baked unique pairs', 'library'),
                        ('plain_frame', 'port C', 'port C')):
    tag = f'shade.vehicle_frame_112_faces_128_normals.{entry}'
    specs.append(dict(id=f'shade.{entry}', function=fn, role=role, suite='shade', fixture='one vehicle: 112 faces, 128 normals, 64 corpora',
                      unit='cycles per vehicle', statistic='median over 64 corpora', tags=[tag],
                      path=['rows', 'vehicle_frame_112_faces_128_normals', entry, 'medianCycles'],
                      compute=lambda p, m, tag=tag: summarize_median(series('shade', tag, p, m))))
for prof in ('zipf256_w6_l12', 'uniform256_w8_l8', 'longtail256_w6_l12', 'few40_w5_l10', 'geometric16_w8_l12'):
    for dec, fn, role in (('fast', 'sh2z_vlc8_decode', 'library'), ('portable', 'sh2z_vlc8_decode_portable', 'library (portable)'),
                          ('plain', 'contributed C', 'contributed C')):
        t1, tn = f'vlc.{prof}.{dec}_decode.call1', f'vlc.{prof}.{dec}_decode.call4096'
        specs.append(dict(id=f'vlc.{prof}.{dec}', function=fn, role=role, suite='vlc',
                          fixture=f'{prof}: 4,096 symbols against 1 symbol, 16 corpora', unit='cycles per symbol',
                          statistic='(mean of the 4,096-symbol calls - mean of the 1-symbol calls) / 4,095', tags=[tn],
                          path=['rows', prof, 'decode', dec, 'cyclesPerSymbol'],
                          compute=lambda p, m, t1=t1, tn=tn: vlc_slope(series('vlc', t1, p, m), series('vlc', tn, p, m), 4096)))

kernels = [make(s) for s in specs]
head = suites['view']['head']
regions = {r['name']: r for r in head['regions']}
result = dict(
    schema=1, tier='MODELLED',
    kind='SH2ZAM placement model: Yabause isolated SH-2 interpreter plus a harness SH7604 cache and region-cost model, not Saturn timing',
    cycleUnit='upstream interpreter per-opcode modeled cycles plus harness stall cycles (SH-2 clocks)',
    calibration=('Region costs are taken from hardware notes of a Saturn port (2026-11-12), which cite Sega\'s Saturn programming FAQ '
                 'for work RAM and an emulator\'s commented bus table for VDP1/VDP2; cartridge figures are ASSUMED there. None is '
                 'measured by this project. The calibration image (saturn/benchmarks/yaul/calibration) times cached and cache-through '
                 'loads from each work RAM and a RAM cartridge, which would check the fill and access costs; it has not been run.'),
    costs=dict(
        hwram=dict(lineFill=regions['hwram']['lineFill'], write=regions['hwram']['write'], basis='hardware notes of a Saturn port, 2026-11-12: fill about 7, write 4 a longword (byte and word writes ASSUMED the same)'),
        lwram=dict(lineFill=regions['lwram']['lineFill'], write=regions['lwram']['write'], basis='hardware notes of a Saturn port, 2026-11-12: 14 a longword (4 x 14 a line), 7 a word (byte ASSUMED as a word)'),
        cart6=dict(lineFill=regions['cart6']['lineFill'], write=regions['cart6']['write'], basis='ASSUMED: 6 clocks a 16-bit beat (low end of the 48-80 line range)'),
        cart10=dict(lineFill=regions['cart10']['lineFill'], write=regions['cart10']['write'], basis='ASSUMED: 10 clocks a 16-bit beat (high end of the 48-80 line range)'),
        vdp1=dict(access=regions['vdp1']['read'], write=regions['vdp1']['write'], basis='hardware notes of a Saturn port, 2026-11-12: 45 per VDP1 VRAM access, any size; cache-through'),
        vdp2=dict(access=regions['vdp2']['read'], write=regions['vdp2']['write'], basis='hardware notes of a Saturn port, 2026-11-12: 40 per VDP2 access, any size; cache-through')),
    cacheModel=('SH7604: 4 KB, 4 ways x 64 sets x 16-byte lines, true LRU, shared by instruction fetches and data, write-through, '
                'no write allocate. Read and fetch misses fill the line and stall for the region\'s fill cost; VDP1/VDP2 reads are '
                'cache-through and stall per access. Writes go through a one-entry write buffer: the next bus access waits for the '
                'previous write to finish. The cache sees the image\'s own addresses (set conflicts follow its layout); classes are '
                'per 16-byte line. Not modelled: the second SH-2 or DMA sharing the bus, SDRAM refresh or page effects, cache-through '
                'reads of work RAM or the cartridge (no figure for single reads), the on-chip divider\'s bus cost (its own model applies), '
                'two-way cache mode.'),
    modes=dict(warm='the cache as the previous call of the same kernel in the same fixture left it (the kernel in a loop with nothing in between); the first call starts empty',
               cold='the cache empty at entry (every line the call touches costs one fill)'),
    placements={p['name']: {k: v for k, v in p.items() if k != 'name'} for p in head['placements']},
    run=dict(evidenceUrl=a.evidence_url,
             interpreterCommit='7e38821dbac265490f115e163c523a939acda759',
             sourceCommits={s: suites[s]['results'].get('sourceCommit') for s in suites},
             images={s: dict(sections=suites[s]['head']['sections'], imageStart=suites[s]['head']['imageStart'],
                             imageEnd=suites[s]['head']['imageEnd']) for s in suites},
             logSHA256={s: sha(getattr(a, f'{s}_log')) for s in suites},
             resultsSHA256={s: sha(getattr(a, f'{s}_results')) for s in suites},
             modelSourceSHA256=sha(Path(__file__).resolve().parent.parent / 'sh2-model' / 'placement_model.h'),
             collectorSourceSHA256=sha(Path(__file__).resolve())),
    kernels=kernels)
a.output.write_text(json.dumps(result, indent=1) + '\n')
print('wrote', a.output, len(kernels), 'kernels')

if a.markdown:
    def fmt(v):
        return f'{v:,.2f}' if (v != int(v) or abs(v) < 1000) else f'{int(v):,}'
    def cell(v, base):
        return f'{fmt(v)} ({v / base:.2f}x)' if base else fmt(v)
    main = ['center.batch', 'center.project', 'center.edgeSteps', 'center.clipNear', 'center.guardInside', 'center.guardSide',
            'center.guardNearSide', 'srcGuard.inside', 'srcGuard.side', 'srcGuard.nearSide', 'srcGuard.roadNear',
            'merge.pixel1', 'merge.rows1', 'shade.fast_light_normals', 'shade.fast_gouraud', 'shade.fast_cull',
            'shade.fast_fog_build', 'shade.fast_fog', 'shade.fast_offset', 'shade.fast_mix', 'shade.fast_frame',
            'shade.fast_frame_baked', 'shade.plain_frame', 'vlc.zipf256_w6_l12.fast', 'vlc.uniform256_w8_l8.fast',
            'vlc.few40_w5_l10.fast', 'vlc.zipf256_w6_l12.portable', 'vlc.zipf256_w6_l12.plain']
    by = {k['id']: k for k in kernels}
    def label(k):
        return f"`{k['function']}`" if k['function'].startswith('sh2z_') else k['function']
    out = ['Best, typical and worst placement. Each cell is warm / cold, then the warm ratio to CPU-only.', '',
           '| Kernel | Fixture | Unit | CPU-only | best | typical | worst |', '| --- | --- | --- | ---: | ---: | ---: | ---: |']
    for i in main:
        k = by[i]; c = k['cpuOnly']; p = k['placed']
        out.append(f"| {label(k)} | {k['id']} | {k['unit']} | {fmt(c)} | " +
                   ' | '.join(f"{fmt(p[n]['warm'])} / {fmt(p[n]['cold'])} ({p[n]['warm'] / c:.2f}x)" for n in ('best', 'typical', 'worst')) + ' |')
    out += ['', 'Other placements, warm cache (ratio to CPU-only):', '',
            '| Fixture | CPU-only | codeLow | cartTables | outVdp1 | outVdp2 |', '| --- | ---: | ---: | ---: | ---: | ---: |']
    for i in main:
        k = by[i]; c = k['cpuOnly']; p = k['placed']
        out.append(f"| {k['id']} | {fmt(c)} | " + ' | '.join(cell(p[n]['warm'], c) for n in ('codeLow', 'cartTables', 'outVdp1', 'outVdp2')) + ' |')
    a.markdown.write_text('\n'.join(out) + '\n')
    print('wrote', a.markdown)
