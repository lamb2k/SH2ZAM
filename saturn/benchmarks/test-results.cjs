/* Synthetic validator tests only. These values are NEVER published. */
const assert=require('node:assert/strict'),{validate}=require('./compile-results.cjs'),m=require('./manifest.json');
const pending={schema:1,fixtureSHA256:m.fixtureSHA256,run:null,records:[]};
assert.equal(validate(pending,m).records.length,0);
const input={...pending,run:{environment:'physical-console',backend:'sh2-dmuls',timerDivisor:128,chunkVertices:32,clockHz:26800000,sourceCommit:'a'.repeat(40),imageSHA256:'b'.repeat(64),compiler:'synthetic test',flags:'test',console:'test fixture',videoMode:'test',cachePolicy:'test',memoryPlacement:'test',capturedAt:'2026-11-13T00:00:00Z',evidenceUrl:'https://example.com/test-only'},records:[{...m.cases[0],invalid:false,maxErrorLSB:7,checksum:'12345678',referenceTicks:Array(21).fill(110),fixedTicks:Array(21).fill(60),overheadTicks:Array(21).fill(10)}]};
let out=validate(input,m).records[0];assert.equal(out.referenceCycles,12800);assert.equal(out.fixedCycles,6400);assert.equal(out.referenceOverFixed,2);
for(const mutate of [x=>x.fixtureSHA256='bad',x=>x.run=null,x=>x.records[0].invalid=true,x=>x.records[0].referenceTicks.pop(),x=>x.records[0].vertices++,x=>x.records.push(x.records[0]),x=>x.run.backend='portable',x=>x.run.evidenceUrl='',x=>x.records[0].fixedTicks.fill(0)]){
  const bad=structuredClone(input);mutate(bad);assert.throws(()=>validate(bad,m));
}
const emu=structuredClone(input);emu.run.environment='emulator';assert.equal(validate(emu,m).run.environment,'emulator');
console.log('PASS: pending results, provenance, matching meshes, overhead, cycles, missing/invalid captures');
