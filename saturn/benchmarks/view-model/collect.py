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
e=raw['edges']
assert e['points']>=200000 and e['mismatches']==0 and e['quads']>=65536 and e['clipMismatches']==0
assert e['projectedPoints']>=262144 and e['projectMismatches']==0
assert len(e['patterns'])==16 and min(e['patterns'])>0,'every near-clip in/out pattern must run'
et=e['timing']
gd=raw['guard']
assert gd['trials']>=65536 and gd['mismatches']==0 and gd['vertices']>=200000,'guard clip must be exact'
assert gd['overflows']>=1 and gd['sizes'][1]==0 and gd['sizes'][2]==0,'overflow path must run; 1 or 2 outputs are impossible'
gt=gd['timing']
assert gt['inside']['meanOutputVertices']==4 and gt['side']['meanOutputVertices']==5
mg=raw['merge']
assert mg['trials']>=16384 and mg['mismatches']==0 and mg['mergedRuns']>=10000,'merge planner must be exact'
assert mg['rejectedCalls']>=4000 and mg['hugeRowStrips']>=1000 and mg['blockedBoundaries']>0
rl=mg['runLengths']
assert len(rl)==31 and rl[1]>0 and rl[3]>0 and sum(rl[4:])>0,'runs of 2, 4 and longer must occur'
mt=mg['timing']; mc=mt['configs']
assert mt['boundaries']==64 and mt['runMask']==[1,2,4]
assert set(mc)=={'pixel0.5','pixel1','pixel2','pixel1Aligned','rows1'}
assert mc['pixel0.5']['meanQuads']>=mc['pixel1']['meanQuads']>=mc['pixel2']['meanQuads'],'a looser tolerance never needs more quads'
for c in mc.values(): assert sum(c['runs'][i]*(1,2,4)[i] for i in range(3))==63*mt['trials'],'runs cover every repetition'
rd=raw['round']
assert rd['mismatches']==0,'view-round-1 must be exact'
assert rd['batchVertices']>=262144 and rd['projectedPoints']>=131072 and rd['steppedPoints']>=100000
assert rd['clipQuads']>=16384 and rd['guardPolygons']>=16384 and rd['guardVertices']>=50000
assert rd['xUp']>0 and rd['yDown']>0,'both differences from view-contract-1 must occur'
rt=rd['timing']
for k in ('batch','project','edgeSteps','clipNear','guardInside','guardSide','guardNearSide'):
    assert set(rt[k]['median'])=={'floorPortable','floorDivu','truncPortable','truncDivu'}
assert rt['batch']['divisions']==2*512*rt['trials'],'round timing batch must cost two divisions per vertex'
gs=raw['guardSrc']
assert gs['mismatches']==0,'view-guard-src-1 must be exact'
assert gs['trials']>=32768 and gs['roadTrials']>=16384 and gs['vertices']>=100000 and gs['roadPolygons']>=10000
assert gs['overflows']>=1 and gs['snapFailures']>=1,'overflow and snap failures must run and write nothing'
assert gs['sizes'][1]==0 and gs['sizes'][2]==0,'1 or 2 outputs are impossible'
gst=gs['timing']
for k in ('inside','side','nearSide','roadNear'):
    assert set(gst[k]['median'])=={'truncPortable','truncDivu','srcPortable','srcDivu'}
assert gst['inside']['meanOutputVertices']==4 and gst['side']['meanOutputVertices']==5

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
      edgesCaptureSourceSHA256=sha((here/'edges.h').read_bytes()),
      guardCaptureSourceSHA256=sha((here/'guard.h').read_bytes()),
      mergeCaptureSourceSHA256=sha((here/'merge.h').read_bytes()),
      roundCaptureSourceSHA256=sha((here/'round.h').read_bytes()),
      guardSrcCaptureSourceSHA256=sha((here/'guardsrc.h').read_bytes()),
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
           'outputs_equal_portable':True},
          {'kernel':'view_project','points':512,'unit':'cycles per point','tier':'MODELLED',
           'portable':et['medianCyclesPerProjectedPoint']['portable'],'divu_stall_model':et['medianCyclesPerProjectedPoint']['divuStallModel'],
           'outputs_equal_oracle':True},
          {'kernel':'view_edge_steps','points':64,'unit':'cycles per point','tier':'MODELLED',
           'portable':et['medianCyclesPerPoint']['portable'],'divu_stall_model':et['medianCyclesPerPoint']['divuStallModel'],
           'one_point_call_cycles':et['medianCyclesOnePointCall'],'outputs_equal_oracle':True},
          {'kernel':'view_clip_near','crossings':2,'unit':'cycles per quad','tier':'MODELLED',
           'portable':et['medianCyclesPerQuad']['portable'],'divu_stall_model':et['medianCyclesPerQuad']['divuStallModel'],
           'outputs_equal_oracle':True}]+
         [{'kernel':'view_clip_guard','shape':s,'band':gt['band'],'attrs':gt['attrs'],'unit':'cycles per quad','tier':'MODELLED',
           'output_vertices':gt[s]['meanOutputVertices'],'portable':gt[s]['medianCycles']['portable'],
           'divu_stall_model':gt[s]['medianCycles']['divuStallModel'],'outputs_equal_oracle':True}
          for s in ('inside','side','nearSide')]+
         [{'kernel':'view_merge_runs','config':k,'mode':mc[k]['mode'],'tol_q8':mc[k]['tolQ8'],'end_masks':mc[k]['endMasks'],
           'boundaries':mt['boundaries'],'run_mask':mt['runMask'],'unit':'cycles per boundary','tier':'MODELLED',
           'cycles_per_boundary':mc[k]['medianCyclesPerBoundary'],'max_cycles_per_boundary':mc[k]['maxCyclesPerBoundary'],
           'mean_quads':mc[k]['meanQuads'],'outputs_equal_oracle':True}
          for k in ('pixel1','pixel0.5','pixel2','pixel1Aligned','rows1')]+
         [{'kernel':'view_round_'+k,'unit':'cycles per vertex' if rt[k]['per']>1 else 'cycles per call',
           'per':rt[k]['per'],'tier':'MODELLED','floor_portable':rt[k]['median']['floorPortable'],
           'floor_divu_stall_model':rt[k]['median']['floorDivu'],'trunc_portable':rt[k]['median']['truncPortable'],
           'trunc_divu_stall_model':rt[k]['median']['truncDivu'],'outputs_equal_oracle':True}
          for k in ('batch','project','edgeSteps','clipNear','guardInside','guardSide','guardNearSide')]+
         [{'kernel':'view_guard_src_'+k,'unit':'cycles per call','band':gst['band'],'attrs':gst[k]['attrs'],'max_out':gst[k]['maxOut'],
           'tier':'MODELLED','output_vertices':gst[k]['meanOutputVertices'],
           'trunc_portable':gst[k]['median']['truncPortable'],'trunc_divu_stall_model':gst[k]['median']['truncDivu'],
           'src_portable':gst[k]['median']['srcPortable'],'src_divu_stall_model':gst[k]['median']['srcDivu'],
           'outputs_equal_oracle':True}
          for k in ('inside','side','nearSide','roadNear')],
    edges=e,guard=gd,merge=mg,round=rd,guardSrc=gs)
a.output.parent.mkdir(parents=True,exist_ok=True)
a.output.write_text(json.dumps(result,indent=1)+'\n')
print('PASS: %d sweep vertices exact on both paths (%d strided); %d divider probes (%d hardware path)'%(
    s['vertices'],s['stride12Vertices'],raw['dividerProbeCount'],raw['dividerProbeHardwarePath']))
print('cycles/vertex at 512 (median): portable %.2f, divu raw %.2f, stall model %.2f, +39/division %.2f'%(
    m['portable'],m['divuRaw'],m['divuStallModel'],m['divuPlusLatencyPerDivision']))
print('view-edges-1: %d stepped points, %d clipped quads, %d projected points exact; project %.2f/%.2f, steps %.2f/%.2f per point, clip %d/%d per quad (portable/divu)'%(
    e['points'],e['quads'],e['projectedPoints'],et['medianCyclesPerProjectedPoint']['portable'],et['medianCyclesPerProjectedPoint']['divuStallModel'],
    et['medianCyclesPerPoint']['portable'],et['medianCyclesPerPoint']['divuStallModel'],et['medianCyclesPerQuad']['portable'],et['medianCyclesPerQuad']['divuStallModel']))
print('view-guard-1: %d polygons, %d output vertices exact (%d overflows, nothing written); quad cycles portable/divu: inside %d/%d, side %d/%d, near+side %d/%d'%(
    gd['trials'],gd['vertices'],gd['overflows'],gt['inside']['medianCycles']['portable'],gt['inside']['medianCycles']['divuStallModel'],
    gt['side']['medianCycles']['portable'],gt['side']['medianCycles']['divuStallModel'],
    gt['nearSide']['medianCycles']['portable'],gt['nearSide']['medianCycles']['divuStallModel']))
print('view-merge-1: %d strips, %d runs exact (%d merged), %d rejected calls wrote nothing; 64-boundary strips, runs {1,2,4}, cycles/boundary (quads): '%(
    mg['trials'],mg['runs'],mg['mergedRuns'],mg['rejectedCalls'])+
    ', '.join('%s %.1f (%.2f)'%(k,mc[k]['medianCyclesPerBoundary'],mc[k]['meanQuads']) for k in ('pixel0.5','pixel1','pixel2','pixel1Aligned','rows1')))
print('view-round-1: %d batch, %d projected, %d stepped, %d near-clip, %d guard vertices exact (sx+1 %d, sy-1 %d); floor/trunc divu: '%(
    rd['batchVertices'],rd['projectedPoints'],rd['steppedPoints'],rd['clipVertices'],rd['guardVertices'],rd['xUp'],rd['yDown'])+
    ', '.join('%s %.2f/%.2f'%(k,rt[k]['median']['floorDivu'],rt[k]['median']['truncDivu']) for k in ('batch','project','edgeSteps','clipNear','guardInside','guardSide','guardNearSide')))
print('view-guard-src-1: %d polygons (%d road quads), %d output vertices exact (%d overflows, %d snap failures, nothing written); '%(
    gs['trials']+gs['roadTrials'],gs['roadTrials'],gs['vertices'],gs['overflows'],gs['snapFailures'])+
    'trunc/src divu cycles: '+', '.join('%s %d/%d'%(k,gst[k]['median']['truncDivu'],gst[k]['median']['srcDivu']) for k in ('inside','side','nearSide','roadNear')))
