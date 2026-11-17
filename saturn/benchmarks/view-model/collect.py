#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (c) 2026 SH2ZAM Contributors.
"""Validate and package the view batch SH-2 model capture. MODELLED tier only:
never console timing, never emulator timing."""
import argparse, hashlib, json, re, subprocess
from pathlib import Path

parser=argparse.ArgumentParser()
parser.add_argument('--raw',type=Path,required=True)
parser.add_argument('--elf',type=Path,required=True)
parser.add_argument('--symbols',type=Path,required=True)
parser.add_argument('--core',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--evidence-url',required=True)
a=parser.parse_args()
raw=json.loads(a.raw.read_text()); elf=a.elf.read_bytes()
sha=lambda b:hashlib.sha256(b).hexdigest()
core_commit=subprocess.check_output(['git','-C',str(a.core),'rev-parse','HEAD'],text=True).strip()
assert core_commit=='7e38821dbac265490f115e163c523a939acda759','Unexpected interpreter version'
subprocess.run(['git','-C',str(a.core),'diff','--exit-code','HEAD','--','yabause/src'],check=True)
ids={m.group(1).decode() for m in re.finditer(rb'(?<![0-9a-f])([0-9a-f]{40})\x00',elf)}
assert len(ids)==1 and raw['sourceCommit'] in ids,'ELF source commit not verified'

assert raw['schema']==1 and raw['dividerLatencyAssumed']==39
assert raw['dividerProbeCount']>=2000 and raw['dividerProbeHardwarePath']>=1000
s=raw['sweep']
assert s['vertices']>=1048576 and s['portableMismatches']==0 and s['divuMismatches']==0
assert s['near']>0 and s['saturatedQuotients']>0 and s['stride12Vertices']>0
t=raw['timing']
assert t['verticesPerBatch']==512 and len(t['batches'])==t['poses']
assert t['near']==0 and t['divisions']==2*512*t['poses'],'timing fixture must cost two divisions per vertex'
m=t['medianCyclesPerVertex']
assert m['divuRaw']<=m['divuStallModel']<=m['divuPlusLatencyPerDivision']

here=Path(__file__).resolve().parent
model=here.parent/'sh2-model'
result=dict(schema=1,kernel='view_batch',tier='MODELLED',sourceCommit=raw['sourceCommit'],
    run=dict(environment='isolated-sh2-interpreter',elfSHA256=sha(elf),symbolSHA256=sha(a.symbols.read_bytes()),
      evidenceUrl=a.evidence_url,interpreterRepository='https://github.com/Yabause/yabause',
      interpreterCommit=core_commit,interpreterSourceSHA256=sha((a.core/'yabause/src/sh2int.c').read_bytes()),
      interpreterModified=False,memoryModel='flat RAM',
      dividerModel='harness code: 39-cycle latency, reads before completion hold the CPU (assumption, not hardware evidence)',
      cycleUnit='upstream interpreter per-opcode modeled cycles',
      includes=['argument and view validation','camera block preparation','all output writes'],
      limitations=['Not physical Saturn timing or full-system emulator timing','No cache or bus wait-state model',
        'No pipeline or multiplier-latency model','Divider behaviour is the harness model, not the console',
        'No interrupts, DMA or second CPU'],
      harnessSourceSHA256=sha((model/'harness.c').read_bytes()),
      dividerModelSourceSHA256=sha((model/'divider_model.h').read_bytes()),
      captureSourceSHA256=sha((here/'capture.h').read_bytes()),
      bridgeSourceSHA256=sha((here/'view_bridge.c').read_bytes()),
      collectorSourceSHA256=sha(Path(__file__).read_bytes())),
    dividerProbes=dict(count=raw['dividerProbeCount'],hardwarePath=raw['dividerProbeHardwarePath'],table=raw['dividerProbes']),
    sweep=s,
    timing={k:v for k,v in t.items()},
    rows=[{'kernel':'view_batch','vertices':512,'path':'portable (libgcc 64-bit divide)',
           'cycles_model':m['portable'],'tier':'MODELLED','outputs_equal_oracle':True},
          {'kernel':'view_batch_divu','vertices':512,'path':'sh2-divu',
           'cycles_model':m['divuRaw'],'cycles_model_stall':m['divuStallModel'],
           'cycles_model_plus_divide_latency':m['divuPlusLatencyPerDivision'],'tier':'MODELLED',
           'outputs_equal_portable':True}])
a.output.parent.mkdir(parents=True,exist_ok=True)
a.output.write_text(json.dumps(result,indent=1)+'\n')
print('PASS: %d sweep vertices exact on both paths (%d strided); %d divider probes (%d hardware path)'%(
    s['vertices'],s['stride12Vertices'],raw['dividerProbeCount'],raw['dividerProbeHardwarePath']))
print('cycles/vertex at 512 (median): portable %.2f, divu raw %.2f, stall model %.2f, +39/division %.2f'%(
    m['portable'],m['divuRaw'],m['divuStallModel'],m['divuPlusLatencyPerDivision']))
