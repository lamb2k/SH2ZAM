/* Evidence integrity and environment-label checks. MIT. */
const assert=require('node:assert/strict');
const {validate,KERNEL_SET}=require('./compile-model-results.cjs');
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
/* Synthetic validator fixture only, never published as benchmark evidence.
 * Once the committed capture is v2, its actual independent samples are used. */
const adoption=structuredClone(data);
if(adoption.schema===1){
  adoption.schema=2;adoption.kernelSet=KERNEL_SET;
  adoption.guards.fullRange.plainAvailable=false;
  adoption.guards.fullRange.plainRejectedWithoutWrites=true;
  for(const row of adoption.records){
    row.plainAvailable=Array(21).fill(true);
    row.plainPreparationCycles=Array(21).fill(17);
    row.plainPreparationInstructions=Array(21).fill(13);
    row.kernelCycles.plain=[...row.kernelCycles.safe];
    row.kernelInstructions.plain=[...row.kernelInstructions.safe];
    row.outputChecksums.plain=[...row.outputChecksums.safe];
    row.medianCycles.plain=row.medianCycles.safe+17;
    row.medianInstructions.plain=row.medianInstructions.safe+13;
    row.medianKernelCycles.plain=row.medianKernelCycles.safe;
    row.referenceOver.plain=row.medianCycles.reference/row.medianCycles.plain;
    row.plainOver=Object.fromEntries(['safe','prepared','mac','dmuls'].map(name=>[name,row.medianCycles.plain/row.medianCycles[name]]));
  }
}
const checked=validate(adoption,manifest);
assert.equal(checked.schema,2);assert.equal(checked.hasPlainBaseline,true);
for(const row of checked.records){
  assert.ok(row.medianCycles.plain>row.medianKernelCycles.plain,'Independent bound proof included');
  assert.equal(row.plainOver.mac,row.medianCycles.plain/row.medianCycles.mac);
}
const adoptionMutations=[
  d=>delete d.records[0].kernelCycles.plain,
  d=>d.records[0].plainPreparationCycles[0]=0,
  d=>d.records[0].plainPreparationInstructions.pop(),
  d=>d.records[0].plainAvailable[0]=false,
  d=>d.records[0].medianCycles.plain=d.records[0].medianKernelCycles.plain,
  d=>d.records[0].plainOver.mac*=2,
  d=>d.records[0].outputChecksums.plain[0]='ffffffff',
  d=>d.guards.fullRange.plainAvailable=true,
  d=>d.guards.fullRange.plainRejectedWithoutWrites=false,
];
for(const mutate of adoptionMutations){const bad=structuredClone(adoption);mutate(bad);assert.throws(()=>validate(bad,manifest));}
console.log('PASS: isolated model identity, independent C baseline proof and ratios, all shared workloads, raw-derived inclusive costs, exact outputs and native/full-range guards');
