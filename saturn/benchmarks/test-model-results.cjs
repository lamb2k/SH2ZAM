/* Evidence integrity and environment-label checks. MIT. */
const assert=require('node:assert/strict');
const {validate}=require('./compile-model-results.cjs');
const data=require('./sh2-model/results.json'),manifest=require('./manifest.json');
const normalized=validate(data,manifest);
assert.equal(normalized.records.length,15);
assert.equal(normalized.run.environment,'isolated-sh2-interpreter');
for(const row of normalized.records){
  assert.ok(row.medianCycles.mac>row.medianKernelCycles.mac,'Plan cost included');
  assert.equal(row.referenceOver.mac,row.medianCycles.reference/row.medianCycles.mac);
}
const mutations=[
  d=>d.run.environment='physical-console',d=>d.run.environment='emulator',
  d=>d.run.memoryModel='physical Saturn',d=>d.run.elfSHA256='',
  d=>d.fixtureSHA256='bad',d=>d.records.pop(),d=>d.records.push(d.records[0]),
  d=>d.records[0].sampleCount=1,d=>d.records[0].candidateMasks[0]=5,
  d=>d.records[0].fixedMismatches=1,d=>d.records[0].maximumWorldError=NaN,
  d=>d.records[0].maximumWorldError=3/65536,d=>d.records[0].kernelCycles.mac[0]=0,
  d=>d.records[0].kernelInstructions.reference.pop(),
  d=>d.records[0].medianCycles.mac=d.records[0].medianKernelCycles.mac,
  d=>d.records[0].referenceOver.mac*=2,
  d=>d.records[0].outputChecksums.mac[0]='ffffffff',
  d=>d.guards.saturationMode.macInstructions=9,d=>d.guards.saturationMode.srSRetained=false,
  d=>d.guards.fullRange.candidateMask=7,d=>d.run.limitations=[],
];
for(const mutate of mutations){const bad=structuredClone(data);mutate(bad);assert.throws(()=>validate(bad,manifest));}
console.log('PASS: isolated model identity, all shared workloads, raw-derived inclusive costs, exact outputs and native fallback guards');
