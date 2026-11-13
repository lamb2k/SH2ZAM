/* Synthetic validator tests only. These values are NEVER published. */
const assert=require('node:assert/strict'),{validate,SCHEMA,KERNEL_SET,PREPARE_ITERATIONS}=require('./compile-results.cjs'),m=require('./manifest.json');
const pending={schema:SCHEMA,fixtureSHA256:m.fixtureSHA256,run:null,records:[]};
assert.equal(validate(pending,m).records.length,0);
assert.equal(validate(pending,m).fixtureSHA256,m.fixtureSHA256);
const input={...pending,run:{environment:'physical-console',kernelSet:KERNEL_SET,backend:'sh2-dmuls',timerDivisor:128,chunkVertices:32,prepareIterations:PREPARE_ITERATIONS,clockHz:26800000,sourceCommit:'a'.repeat(40),imageSHA256:'b'.repeat(64),compiler:'synthetic test',flags:'test',console:'test fixture',videoMode:'test',cachePolicy:'test',memoryPlacement:'test',capturedAt:'2026-11-13T00:00:00Z',evidenceUrl:'https://example.com/test-only'},records:[{...m.cases[0],invalid:false,maxErrorLSB:2,fixedMismatches:0,checksum:'12345678',referenceChecksum:'87654321',generalChecksum:'11223344',optimizedChecksum:'11223344',macChecksum:'11223344',dmulsChecksum:'11223344',referenceTicks:Array(21).fill(110),generalTicks:Array(21).fill(210),optimizedTicks:Array(21).fill(60),macTicks:Array(21).fill(30),dmulsTicks:Array(21).fill(50),overheadTicks:Array(21).fill(10),prepareTicks:Array(21).fill(144),prepareOverheadTicks:Array(21).fill(16),candidateMasks:Array(21).fill(7)}]};
let out=validate(input,m).records[0];
assert.equal(out.referenceCycles,12800);assert.equal(out.generalCycles,25600);
assert.equal(out.prepareCycles,256);assert.equal(out.optimizedKernelCycles,6400);assert.equal(out.optimizedCycles,6656);
assert.equal(out.macKernelCycles,2560);assert.equal(out.macCycles,2816);
assert.equal(out.dmulsKernelCycles,5120);assert.equal(out.dmulsCycles,5376);
assert.equal(out.referenceOverGeneral,0.5);assert.equal(out.referenceOverOptimized,12800/6656);assert.equal(out.generalOverOptimized,25600/6656);
assert.equal(out.optimizedMs,6656/26800000*1000);assert.equal(out.prepareMs,256/26800000*1000);
assert.equal(out.preparedAvailable,true);assert.equal(out.macAvailable,true);assert.equal(out.dmulsAvailable,true);
assert.equal(out.referenceOverMac,12800/2816);assert.equal(out.optimizedOverDmuls,6656/5376);
const mutations=[
  x=>x.schema=1,x=>x.schema=2,x=>x.fixtureSHA256='bad',x=>x.run=null,x=>x.records[0].invalid=true,
  x=>x.records[0].referenceTicks.pop(),x=>x.records[0].vertices++,x=>x.records.push(x.records[0]),
  x=>x.run.backend='portable',x=>x.run.kernelSet='float-general-batch-v1',x=>delete x.run.kernelSet,x=>x.run.evidenceUrl='',
  x=>x.run.prepareIterations=1,x=>delete x.run.prepareIterations,
  x=>x.records[0].generalTicks.fill(0),x=>x.records[0].optimizedTicks.fill(10),x=>x.records[0].overheadTicks.fill(200),
  x=>delete x.records[0].generalTicks,x=>delete x.records[0].optimizedTicks,x=>x.records[0].optimizedTicks[3]=1.5,
  x=>delete x.records[0].macTicks,x=>x.records[0].dmulsTicks[5]=Number.MAX_SAFE_INTEGER,
  x=>x.records[0].fixedMismatches=1,x=>delete x.records[0].fixedMismatches,
  x=>x.records[0].optimizedChecksum='ffeeddcc',x=>delete x.records[0].referenceChecksum,
  x=>x.records[0].macChecksum='ffeeddcc',x=>delete x.records[0].dmulsChecksum,
  x=>x.records[0].maxErrorLSB=67,x=>x.records[0].maxErrorLSB=-1,x=>x.records[0].maxErrorLSB=NaN,
  x=>delete x.records[0].prepareTicks,x=>x.records[0].prepareTicks[1]=65536,
  x=>x.records[0].prepareOverheadTicks[0]=145,x=>x.records[0].prepareTicks[0]=-1,
  x=>delete x.records[0].candidateMasks,x=>x.records[0].candidateMasks.pop(),
  x=>x.records[0].candidateMasks[0]=8,x=>x.records[0].candidateMasks[0]=6,
];
for(const mutate of mutations){const bad=structuredClone(input);mutate(bad);assert.throws(()=>validate(bad,m));}
for(const schema of [1,2])assert.throws(()=>validate({...pending,schema},m));
const emu=structuredClone(input);emu.run.environment='emulator';assert.equal(validate(emu,m).run.environment,'emulator');
/* Include setup per pose BEFORE taking the median, with fractional FRT ticks
 * allowed after the 64-call average. Adding separate medians is incorrect. */
const varied=structuredClone(input),row=varied.records[0];
row.overheadTicks=Array.from({length:21},(_,i)=>i*100);
row.optimizedTicks=row.overheadTicks.map((x,i)=>x+(i<10?1:100));
row.macTicks=row.overheadTicks.map((x,i)=>x+(i<10?1:100));
row.dmulsTicks=row.overheadTicks.map((x,i)=>x+(i<10?1:100));
row.generalTicks=row.overheadTicks.map(x=>x+40);
row.referenceTicks=row.overheadTicks.map(x=>x+30);
row.prepareOverheadTicks=Array(21).fill(0);
row.prepareTicks=Array.from({length:21},(_,i)=>(i<11?100:0)*64+1);
out=validate(varied,m).records[0];
assert.equal(out.optimizedKernelCycles,100*128);
assert.equal(out.prepareCycles,(100+1/64)*128);
assert.equal(out.optimizedCycles,(101+1/64)*128);
assert.notEqual(out.optimizedCycles,out.optimizedKernelCycles+out.prepareCycles);
assert.equal(out.macCycles,out.optimizedCycles);assert.equal(out.dmulsCycles,out.optimizedCycles);
/* Zero resolved setup is permitted; negative setup and invented epsilons are
 * not. Raw values remain available for timer-resolution review. */
const zeroSetup=structuredClone(input);zeroSetup.records[0].prepareTicks=Array(21).fill(16);
out=validate(zeroSetup,m).records[0];assert.equal(out.prepareCycles,0);assert.equal(out.optimizedCycles,6400);
/* Fallback output still requires parity, but no active-candidate timing is
 * published when any pose used the fallback. */
const fallback=structuredClone(input);fallback.records[0].candidateMasks[8]=5;
out=validate(fallback,m).records[0];
assert.equal(out.macAvailable,false);assert.equal(out.macCycles,null);assert.equal(out.macKernelCycles,null);
assert.equal(out.macMs,null);assert.equal(out.referenceOverMac,null);assert.equal(out.dmulsAvailable,true);
assert.deepEqual(out.macTicks,input.records[0].macTicks);
fallback.records[0].candidateMasks.fill(0);out=validate(fallback,m).records[0];
assert.equal(out.preparedAvailable,false);assert.equal(out.dmulsCycles,null);assert.equal(out.optimizedCycles,6656);
console.log('PASS: five-kernel schema, four-way fixed parity, inclusive setup, fallback availability, provenance, per-pose medians; legacy captures rejected');
