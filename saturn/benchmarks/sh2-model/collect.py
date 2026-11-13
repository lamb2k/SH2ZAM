#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
"""Validate and package isolated SH-2 interpreter evidence, never console timing."""
import argparse, hashlib, json, math, re, statistics, subprocess
from pathlib import Path

parser=argparse.ArgumentParser()
parser.add_argument('--raw',type=Path,required=True)
parser.add_argument('--elf',type=Path,required=True)
parser.add_argument('--symbols',type=Path,required=True)
parser.add_argument('--core',type=Path,required=True)
parser.add_argument('--manifest',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--evidence-url',required=True)
parser.add_argument('--source-commit')
a=parser.parse_args()
raw=json.loads(a.raw.read_text()); manifest=json.loads(a.manifest.read_text()); elf=a.elf.read_bytes()
sha=lambda b:hashlib.sha256(b).hexdigest()
core_commit=subprocess.check_output(['git','-C',str(a.core),'rev-parse','HEAD'],text=True).strip()
expected_core='7e38821dbac265490f115e163c523a939acda759'
assert core_commit==expected_core,'Unexpected interpreter version'
subprocess.run(['git','-C',str(a.core),'diff','--exit-code','HEAD','--','yabause/src'],check=True)
ids={m.group(1).decode() for m in re.finditer(rb'(?<![0-9a-f])([0-9a-f]{40})\x00',elf)}
source_commit=a.source_commit or (next(iter(ids)) if len(ids)==1 else None)
assert source_commit and re.fullmatch('[0-9a-f]{40}',source_commit) and source_commit in ids,'ELF source commit not verified'
assert manifest['fixtureSHA256'].encode() in elf,'Fixture hash is not embedded in this ELF'
assert raw['schema']==2,'Expected independent plain-fixed baseline capture'
assert len(raw['cases'])==len(manifest['cases'])==15
assert raw['fixedMismatches']==0 and math.isfinite(raw['maximumFloatError'])
assert raw['maximumFloatError']<=2/65536
assert raw['guards']['saturationMode']['candidateMask']==5
assert raw['guards']['saturationMode']['macInstructions']==0
assert raw['guards']['saturationMode']['fixedMismatches']==0
assert raw['guards']['saturationMode']['srSRetained'] is True
assert raw['guards']['fullRange']['candidateMask']==0
assert raw['guards']['fullRange']['macInstructions']==0
assert raw['guards']['fullRange']['fixedMismatches']==0
assert raw['guards']['fullRange']['plainAvailable'] is False
assert raw['guards']['fullRange']['plainRejectedWithoutWrites'] is True
mapping={'reference':'float','safe':'safe','prepared':'prepared','mac':'mac','dmuls':'dmuls','plain':'plain'}
records=[]
for c,expected in zip(raw['cases'],manifest['cases']):
    assert (c['scene'],c['detail'],c['vertices'])==(expected['scene'],expected['detail'],expected['vertices'])
    poses=c['poses']; assert len(poses)==21
    assert all(p['availability']==7 and p['plainAvailable'] is True and p['fixedMismatches']==0 for p in poses)
    assert all(math.isfinite(p['maxFloatError']) and p['maxFloatError']<=2/65536 for p in poses)
    for p in poses:
        assert all(p['checksums'][i]==p['checksums'][1] for i in range(2,6))
        assert p['paths']['mac']['macInstructions']==9*c['vertices']
        assert p['paths']['dmuls']['dmulsInstructions']==9*c['vertices']
        assert p['paths']['prepared']['dmulsInstructions']==9*c['vertices']
        assert p['paths']['plain']['dmulsInstructions']==9*c['vertices']
        assert p['paths']['plain']['macInstructions']==0
    cycles={k:[p['paths'][v]['cycles'] for p in poses] for k,v in mapping.items()}
    instructions={k:[p['paths'][v]['instructions'] for p in poses] for k,v in mapping.items()}
    setup=[p['plan']['cycles'] for p in poses]; setup_i=[p['plan']['instructions'] for p in poses]
    plain_setup=[p['plainPlan']['cycles'] for p in poses]; plain_setup_i=[p['plainPlan']['instructions'] for p in poses]
    inclusive={k:statistics.median(x+(setup[i] if k in ['prepared','mac','dmuls'] else plain_setup[i] if k=='plain' else 0) for i,x in enumerate(v)) for k,v in cycles.items()}
    inclusive_i={k:statistics.median(x+(setup_i[i] if k in ['prepared','mac','dmuls'] else plain_setup_i[i] if k=='plain' else 0) for i,x in enumerate(v)) for k,v in instructions.items()}
    records.append(dict(scene=c['scene'],detail=c['detail'],vertices=c['vertices'],sampleCount=21,
        poses=[{'ax':p['ax'],'ay':p['ay']} for p in poses],candidateMasks=[p['availability'] for p in poses],
        planPreparationCycles=setup,planPreparationInstructions=setup_i,
        plainPreparationCycles=plain_setup,plainPreparationInstructions=plain_setup_i,
        plainAvailable=[p['plainAvailable'] for p in poses],
        kernelCycles=cycles,kernelInstructions=instructions,
        kernelDataReads={k:[p['paths'][v]['dataReads'] for p in poses] for k,v in mapping.items()},
        kernelDataWrites={k:[p['paths'][v]['dataWrites'] for p in poses] for k,v in mapping.items()},
        macInstructions={k:[p['paths'][v]['macInstructions'] for p in poses] for k,v in mapping.items()},
        dmulsInstructions={k:[p['paths'][v]['dmulsInstructions'] for p in poses] for k,v in mapping.items()},
        outputChecksums={k:[p['checksums'][i] for p in poses] for i,k in enumerate(mapping)},
        maximumWorldError=max(p['maxFloatError'] for p in poses),fixedMismatches=0,
        medianCycles=inclusive,medianKernelCycles={k:statistics.median(v) for k,v in cycles.items()},
        medianInstructions=inclusive_i,referenceOver={k:inclusive['reference']/inclusive[k] for k in mapping if k!='reference'},
        plainOver={k:inclusive['plain']/inclusive[k] for k in ['safe','prepared','mac','dmuls']},
        meshPreparation=c['meshPreparation']))
source_dir=Path(__file__).resolve().parent
result=dict(schema=2,kernelSet='float-plain-fixed-safe-prepared-fused-sh2-model-v2',fixtureSHA256=manifest['fixtureSHA256'],
    run=dict(environment='isolated-sh2-interpreter',sourceCommit=source_commit,elfSHA256=sha(elf),
      symbolSHA256=sha(a.symbols.read_bytes()),evidenceUrl=a.evidence_url,
      interpreterRepository='https://github.com/Yabause/yabause',interpreterCommit=core_commit,
      interpreterSourceSHA256=sha((a.core/'yabause/src/sh2int.c').read_bytes()),
      interpreterSource='yabause/src/sh2int.c',interpreterModified=False,memoryModel='flat RAM',
      wholeBatch=True,cycleUnit='upstream interpreter per-opcode modeled cycles',
      includes=['normal range wrapper','all output writes','one plan preparation per prepared/MAC/DMULS batch','one independent bound proof and matrix snapshot per plain-fixed batch'],
      excludes=['input conversion and mesh scan','matrix construction and float conversion','precision checks','host execution overhead'],
      limitations=['Not physical Saturn timing or full-system emulator timing','No cache or bus wait-state model','No pipeline or multiplier-contention model','No DMA, interrupts, timers or peripheral integration','No BIOS or boot sequence executed'],
      harnessSourceSHA256=sha((source_dir/'harness.c').read_bytes()),collectorSourceSHA256=sha(Path(__file__).read_bytes())),
    guards=raw['guards'],checkedComponentsPerFixedPath=raw['checkedComponentsPerFixedPath'],
    fixedMismatches=0,maximumWorldError=raw['maximumFloatError'],records=records)
a.output.parent.mkdir(parents=True,exist_ok=True)
a.output.write_text(json.dumps(result,separators=(',',':'))+'\n')
for r in records:
    print(r['scene'],r['detail'],r['vertices'],r['medianCycles'],r['referenceOver'])
print('PASS: 315 poses; %s components per fixed path; exact independent plain C and library outputs; native S=1/full-range fallbacks; plain full-range rejection'%raw['checkedComponentsPerFixedPath'])
