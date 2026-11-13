/* Synthetic validator tests only. These values are NEVER published. */
const assert=require('node:assert/strict'),{validate,SCHEMA,KERNEL_SET}=require('./compile-results.cjs'),m=require('./manifest.json');
const pending={schema:SCHEMA,fixtureSHA256:m.fixtureSHA256,run:null,records:[]};
assert.equal(validate(pending,m).records.length,0);
assert.equal(validate(pending,m).fixtureSHA256,m.fixtureSHA256);
const input={...pending,run:{environment:'physical-console',kernelSet:KERNEL_SET,backend:'sh2-dmuls',timerDivisor:128,chunkVertices:32,clockHz:26800000,sourceCommit:'a'.repeat(40),imageSHA256:'b'.repeat(64),compiler:'synthetic test',flags:'test',console:'test fixture',videoMode:'test',cachePolicy:'test',memoryPlacement:'test',capturedAt:'2026-11-13T00:00:00Z',evidenceUrl:'https://example.com/test-only'},records:[{...m.cases[0],invalid:false,maxErrorLSB:2,fixedMismatches:0,checksum:'12345678',referenceChecksum:'87654321',generalChecksum:'11223344',optimizedChecksum:'11223344',referenceTicks:Array(21).fill(110),generalTicks:Array(21).fill(210),optimizedTicks:Array(21).fill(60),overheadTicks:Array(21).fill(10)}]};
let out=validate(input,m).records[0];
assert.equal(out.referenceCycles,12800);assert.equal(out.generalCycles,25600);assert.equal(out.optimizedCycles,6400);
assert.equal(out.referenceOverGeneral,0.5);assert.equal(out.referenceOverOptimized,2);assert.equal(out.generalOverOptimized,4);
assert.equal(out.optimizedMs,6400/26800000*1000);
const mutations=[
  x=>x.schema=1,x=>x.fixtureSHA256='bad',x=>x.run=null,x=>x.records[0].invalid=true,
  x=>x.records[0].referenceTicks.pop(),x=>x.records[0].vertices++,x=>x.records.push(x.records[0]),
  x=>x.run.backend='portable',x=>x.run.kernelSet='old-float-fixed',x=>delete x.run.kernelSet,x=>x.run.evidenceUrl='',
  x=>x.records[0].generalTicks.fill(0),x=>x.records[0].optimizedTicks.fill(10),x=>x.records[0].overheadTicks.fill(200),
  x=>delete x.records[0].generalTicks,x=>delete x.records[0].optimizedTicks,x=>x.records[0].optimizedTicks[3]=1.5,
  x=>x.records[0].fixedMismatches=1,x=>delete x.records[0].fixedMismatches,
  x=>x.records[0].optimizedChecksum='ffeeddcc',x=>delete x.records[0].referenceChecksum,
  x=>x.records[0].maxErrorLSB=67,x=>x.records[0].maxErrorLSB=-1,x=>x.records[0].maxErrorLSB=NaN,
];
for(const mutate of mutations){const bad=structuredClone(input);mutate(bad);assert.throws(()=>validate(bad,m));}
const oldPending={...pending,schema:1};assert.throws(()=>validate(oldPending,m));
const emu=structuredClone(input);emu.run.environment='emulator';assert.equal(validate(emu,m).run.environment,'emulator');
/* Median must follow per-pose overhead subtraction, not independent raw medians. */
const varied=structuredClone(input);
varied.records[0].overheadTicks=Array.from({length:21},(_,i)=>i*100);
varied.records[0].optimizedTicks=varied.records[0].overheadTicks.map((x,i)=>x+21-i);
varied.records[0].generalTicks=varied.records[0].overheadTicks.map(x=>x+40);
varied.records[0].referenceTicks=varied.records[0].overheadTicks.map(x=>x+30);
assert.equal(validate(varied,m).records[0].optimizedCycles,11*128);
console.log('PASS: three-kernel schema, exact parity, precision limit, provenance, overhead-adjusted medians; legacy captures rejected');
