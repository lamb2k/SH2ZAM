/* Strict publication gate for exact SH-2 workload evidence.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
'use strict';
const fs = require('node:fs'), path = require('node:path'), crypto = require('node:crypto');
const KERNEL_SET = 'exact-workloads-sh2-model-v1';
const BASELINE_COMMIT = '3374d77c4625749cc932cafa4385ad18db811e9b';
const CORE_COMMIT = '7e38821dbac265490f115e163c523a939acda759';
const BASELINE_FILES = ['sh2zam.c','sh2zam_angles.c','sh2zam_trig.c',
  'sh2zam_transform.c','sh2zam_geometry.c','sh2zam_internal.h'];
const META_KEYS = ['id','group','profile','size','queries','outputWords','paths'];
const COST_KEYS = ['cycles','instructions','dataReads','dataWrites','macInstructions','dmulsInstructions'];
const FLAGS = ['-m2','-mb','-O2','-fno-fast-math','-ffp-contract=off','-fno-lto'];
const LIMITATIONS = [
  'Not physical Saturn timing or full-system emulator timing',
  'No cache or bus wait-state model',
  'No pipeline or multiplier-contention model',
  'No DMA, interrupts, timers or peripheral integration',
  'One SH-2 CPU; no second-CPU scheduling or shared-memory contention',
  'No conversion to milliseconds, frame rate or clock-frequency estimates'
];
const INCLUDES = ['identical inputs and all observable result writes',
  'normal validation and exact fallbacks',
  'one quaternion normalization/matrix construction per composed or batch rotation',
  'input scan and plan qualification for native rotation composition',
  'one spatial index build plus all 128 queries per spatial sample'];
const EXCLUDES = ['fixture construction','output equality and SR.S checks','host execution overhead'];
const HASH_KEYS = ['elfSHA256','symbolSHA256','rawCaptureSHA256','interpreterSourceSHA256',
  'fixtureManifestSHA256','bridgeSourceSHA256','standaloneSourceSHA256','casesSourceSHA256',
  'suiteSourceSHA256','harnessSourceSHA256','captureSourceSHA256','collectorSourceSHA256',
  'validatorSourceSHA256','baselineRenameSHA256','baselineApiSHA256'];
const SPECS = {
  rotation: {sizes:[1,8,32,128,512],profiles:['bounded','tiny','full-range','identity','zero'],
    paths:['scalar','composed','composed_mac','batch','mac']},
  normalize: {sizes:[32],profiles:['bounded','tiny','full-range'],paths:['prior','current']},
  atan2: {sizes:[32],profiles:['bounded','tiny','full-range'],paths:['prior','current']},
  atan: {sizes:[32],profiles:['bounded','tiny','full-range'],paths:['prior','current']},
  spatial: {sizes:[128,512],profiles:['sparse','mixed','dense','crowded','full-range'],
    paths:['api_brute','inline_brute','index']}
};
const REQUIRED_LIBRARY_SOURCES = [
  ...BASELINE_FILES.map(name=>'saturn/src/'+name),
  'saturn/src/sh2zam_prepared.c','saturn/src/sh2zam_prepared_internal.h',
  'saturn/src/sh2zam_batch_sh2.c','saturn/src/sh2zam_reuse.c',
  'saturn/src/sh2zam_rotation_batch.c','saturn/src/sh2zam_spatial.c',
  'saturn/include/sh2zam/sh2zam.h','saturn/include/sh2zam/batch_experimental.h',
  'saturn/include/sh2zam/reuse_experimental.h','saturn/include/sh2zam/rotation_batch.h',
  'saturn/include/sh2zam/spatial.h'
];
const sha = bytes => crypto.createHash('sha256').update(bytes).digest('hex');
const median = values => [...values].sort((a,b)=>a-b)[Math.floor(values.length/2)];
function requireThat(condition,message) { if (!condition) throw new Error(message); }
function exactKeys(value,keys,label) {
  requireThat(value!==null && typeof value==='object' && !Array.isArray(value) &&
    Object.keys(value).length===keys.length && keys.every(key=>Object.hasOwn(value,key)),
    'Unexpected/missing '+label+' fields');
}
function safeInteger(value,minimum,label) {
  requireThat(Number.isSafeInteger(value) && value>=minimum,'Invalid '+label+' count');
}
function hash(value,label) {
  requireThat(typeof value==='string' && /^[a-f0-9]{64}$/.test(value),'Missing/invalid '+label+' hash');
}
function sameList(value,expected,label) {
  requireThat(Array.isArray(value) && value.length===expected.length &&
    expected.every((entry,i)=>value[i]===entry),'Wrong '+label);
}
function equalNumber(actual,expected,label) {
  requireThat(typeof actual==='number' && Number.isFinite(actual) &&
    Math.abs(actual-expected)<=1e-11*Math.max(1,Math.abs(expected)),'Derived metric differs: '+label);
}
function baselinePaths(group) {
  return group==='rotation' ? ['scalar','composed','composed_mac'] :
    group==='spatial' ? ['api_brute','inline_brute'] : ['prior'];
}
function validateManifest(manifest) {
  exactKeys(manifest,['schema','samplesPerCase','cases'],'fixture manifest');
  requireThat(manifest.schema===1 && manifest.samplesPerCase===7,'Wrong fixture schema or sample count');
  requireThat(Array.isArray(manifest.cases) && manifest.cases.length===44,'Expected all 44 workload cases');
  const ids=new Set(), coverage=new Set();
  for (const fixture of manifest.cases) {
    exactKeys(fixture,META_KEYS,'fixture case');
    requireThat(typeof fixture.id==='string' && /^[a-z][a-z0-9_.:-]{0,95}$/.test(fixture.id) &&
      !ids.has(fixture.id),'Invalid/duplicate fixture ID');
    ids.add(fixture.id);
    requireThat(Object.hasOwn(SPECS,fixture.group),'Unknown workload group');
    const spec=SPECS[fixture.group];
    safeInteger(fixture.size,1,'fixture size');
    safeInteger(fixture.queries,0,'fixture queries');
    safeInteger(fixture.outputWords,1,'fixture outputWords');
    requireThat(spec.sizes.includes(fixture.size) && spec.profiles.includes(fixture.profile),
      'Wrong fixture size or profile');
    sameList(fixture.paths,spec.paths,'fixture path order');
    requireThat(fixture.queries===(fixture.group==='spatial'?128:0),'Wrong fixture query count');
    const words=fixture.group==='spatial' ? fixture.queries*(Math.ceil(fixture.size/32)+1) :
      fixture.size*(['rotation','normalize'].includes(fixture.group)?3:1);
    requireThat(fixture.outputWords===words,'Wrong fixture output width');
    const key=fixture.group+':'+fixture.profile+':'+fixture.size;
    requireThat(!coverage.has(key),'Duplicate fixture coverage');
    coverage.add(key);
  }
  for (const [group,spec] of Object.entries(SPECS))
    for (const profile of spec.profiles) for (const size of spec.sizes)
      requireThat(coverage.has(group+':'+profile+':'+size),'Missing fixture coverage');
  return manifest.cases;
}
function validate(data) {
  exactKeys(data,['schema','kernelSet','comparison','baseline','run','samplesPerCase',
    'checkedOutputWords','checkedComparisons','mismatches','records'],'dataset');
  requireThat(data.schema===1 && data.kernelSet===KERNEL_SET,'Wrong native workload model schema');
  exactKeys(data.comparison,['priorLibraryCommit','rotationBaselines','spatialBaselines',
    'scalarBaseline','claimsUniversalTenfoldGain'],'comparison');
  requireThat(data.comparison.priorLibraryCommit===BASELINE_COMMIT &&
    data.comparison.scalarBaseline==='prior' && data.comparison.claimsUniversalTenfoldGain===false,
    'Wrong baseline or unsupported universal speedup claim');
  sameList(data.comparison.rotationBaselines,baselinePaths('rotation'),'rotation baselines');
  sameList(data.comparison.spatialBaselines,baselinePaths('spatial'),'spatial baselines');
  exactKeys(data.baseline,['sourceCommit','manifestSHA256','files'],'baseline');
  requireThat(data.baseline.sourceCommit===BASELINE_COMMIT,'Wrong frozen prior commit');
  hash(data.baseline.manifestSHA256,'baseline manifest');
  const priorBytes=fs.readFileSync(path.join(__dirname,'prior/manifest.json'));
  const prior=JSON.parse(priorBytes);
  exactKeys(prior,['sourceCommit','files'],'prior manifest');
  requireThat(prior.sourceCommit===BASELINE_COMMIT && sha(priorBytes)===data.baseline.manifestSHA256,
    'Prior manifest identity differs');
  exactKeys(prior.files,BASELINE_FILES,'prior manifest sources');
  exactKeys(data.baseline.files,BASELINE_FILES,'prior sources');
  for (const name of BASELINE_FILES) {
    hash(data.baseline.files[name],'prior '+name);
    requireThat(data.baseline.files[name]===prior.files[name] &&
      sha(fs.readFileSync(path.join(__dirname,'prior',name)))===prior.files[name],
      'Frozen prior bytes differ: '+name);
  }
  const fixtureBytes=fs.readFileSync(path.join(__dirname,'manifest.json'));
  const cases=validateManifest(JSON.parse(fixtureBytes));
  const run=data.run;
  exactKeys(run,['environment','sourceCommit','compiler','compileFlags','toolchainVerifiedInELF',
    'evidenceUrl','interpreterRepository','interpreterCommit','interpreterSource','interpreterModified',
    'memoryModel','cpuCount','wholeBatch','setupIncluded','cycleUnit','hardwareMeasured','browserTimingUsed',
    'hostResults','includes','excludes','limitations','librarySourceSHA256','baselineWrapperSHA256',...HASH_KEYS],'run');
  requireThat(run.environment==='isolated-sh2-interpreter' && run.memoryModel==='flat RAM' && run.cpuCount===1 &&
    run.wholeBatch===true && run.setupIncluded===true && run.cycleUnit==='upstream interpreter per-opcode modeled cycles',
    'Expected one isolated SH-2 instruction model with whole-batch setup included');
  requireThat(run.hardwareMeasured===false && run.browserTimingUsed===false && run.hostResults===false,
    'This evidence contains native instruction-model results only');
  requireThat(typeof run.sourceCommit==='string' && /^[a-f0-9]{40}$/.test(run.sourceCommit),'Missing source commit');
  requireThat(run.interpreterRepository==='https://github.com/Yabause/yabause' && run.interpreterCommit===CORE_COMMIT &&
    run.interpreterSource==='yabause/src/sh2int.c' && run.interpreterModified===false,'Wrong or modified interpreter');
  requireThat(typeof run.compiler==='string' && run.compiler.trim() && run.toolchainVerifiedInELF===true,
    'Missing ELF-verified compiler provenance');
  requireThat(typeof run.compileFlags==='string','Missing compiler flags');
  const flags=run.compileFlags.trim().split(/\s+/);
  requireThat(FLAGS.every(flag=>flags.includes(flag)) && !flags.includes('-ffast-math') && !flags.includes('-flto'),
    'Unexpected native compile configuration');
  requireThat(typeof run.evidenceUrl==='string' &&
    /^https:\/\/github\.com\/[A-Za-z0-9_.-]+\/[A-Za-z0-9_.-]+\/actions\/(?:runs\/[0-9]+|workflows\/[^\s?#]+)$/.test(run.evidenceUrl),
    'Missing public GitHub Actions evidence');
  for (const key of HASH_KEYS) hash(run[key],key);
  requireThat(run.fixtureManifestSHA256===sha(fixtureBytes),'Fixture manifest identity differs');
  requireThat(run.librarySourceSHA256!==null && typeof run.librarySourceSHA256==='object' &&
    !Array.isArray(run.librarySourceSHA256),'Missing library source hashes');
  for (const required of REQUIRED_LIBRARY_SOURCES)
    requireThat(Object.hasOwn(run.librarySourceSHA256,required),'Missing library source: '+required);
  for (const [name,value] of Object.entries(run.librarySourceSHA256)) {
    requireThat(/^saturn\/(?:src\/[a-zA-Z0-9_]+\.[ch]|include\/sh2zam\/[a-zA-Z0-9_]+\.h)$/.test(name),
      'Invalid library source path');
    hash(value,'library '+name);
  }
  const wrappers=BASELINE_FILES.filter(name=>name.endsWith('.c')).map(name=>'prior_'+name);
  exactKeys(run.baselineWrapperSHA256,wrappers,'prior wrapper sources');
  for (const name of wrappers) hash(run.baselineWrapperSHA256[name],'wrapper '+name);
  sameList(run.includes,INCLUDES,'measured scope');
  sameList(run.excludes,EXCLUDES,'excluded scope');
  sameList(run.limitations,LIMITATIONS,'model limitations');
  requireThat(data.samplesPerCase===7,'Expected seven changing-input samples per case');
  requireThat(data.mismatches===0,'Native output mismatches must be zero');
  requireThat(Array.isArray(data.records) && data.records.length===cases.length,'All 44 workload records are required');
  const expected=new Map(cases.map(fixture=>[fixture.id,fixture])), seen=new Set();
  let checked=0, comparisons=0;
  for (const record of data.records) {
    exactKeys(record,[...META_KEYS,'samples','guards','sampleCount','setupIncluded','medianCycles',
      'cyclesPerItem','relativeTo','checkedOutputWords','checkedComparisons'],'record');
    requireThat(expected.has(record.id) && !seen.has(record.id),'Unknown/duplicate workload '+record.id);
    seen.add(record.id);
    const fixture=expected.get(record.id), label=record.id;
    for (const key of META_KEYS.filter(key=>key!=='paths'))
      requireThat(record[key]===fixture[key],'Wrong fixture metadata '+label+' '+key);
    sameList(record.paths,fixture.paths,'record path order '+label);
    exactKeys(record.guards,['srSRetained','srSOutputMatch','noMacWithS'],'native guards');
    requireThat(Object.values(record.guards).every(value=>value===true),'SR.S guard failed '+label);
    requireThat(record.sampleCount===7 && record.setupIncluded===true &&
      Array.isArray(record.samples) && record.samples.length===7,'Incomplete samples or excluded setup '+label);
    for (const sample of record.samples) {
      exactKeys(sample,['inputChecksum','paths','mismatches'],'sample');
      requireThat(typeof sample.inputChecksum==='string' && /^[a-f0-9]{8}$/.test(sample.inputChecksum),
        'Invalid native input checksum '+label);
      requireThat(sample.mismatches===0,'Native sample outputs differ '+label);
      exactKeys(sample.paths,fixture.paths,'sample paths');
      const outputChecksums=new Set();
      for (const name of fixture.paths) {
        const cost=sample.paths[name];
        exactKeys(cost,[...COST_KEYS,'checksum'],'path cost');
        for (const metric of COST_KEYS)
          safeInteger(cost[metric],['cycles','instructions'].includes(metric)?1:0,label+' '+name+' '+metric);
        requireThat(cost.macInstructions+cost.dmulsInstructions<=cost.instructions,'Impossible opcode counters '+label);
        requireThat(typeof cost.checksum==='string' && /^[a-f0-9]{8}$/.test(cost.checksum),
          'Invalid native output checksum '+label);
        outputChecksums.add(cost.checksum);
      }
      requireThat(outputChecksums.size===1,'Native path output checksums differ '+label);
    }
    exactKeys(record.medianCycles,fixture.paths,'median cycles');
    exactKeys(record.cyclesPerItem,fixture.paths,'cycles per item');
    const medians={};
    const denominator=record.group==='spatial'?record.queries:record.size;
    for (const name of fixture.paths) {
      medians[name]=median(record.samples.map(sample=>sample.paths[name].cycles));
      requireThat(record.medianCycles[name]===medians[name],'Median differs from raw samples '+label+' '+name);
      equalNumber(record.cyclesPerItem[name],medians[name]/denominator,'cycles per item '+label+' '+name);
    }
    const baselines=baselinePaths(record.group);
    exactKeys(record.relativeTo,baselines,'relative baselines');
    for (const baseline of baselines) {
      exactKeys(record.relativeTo[baseline],fixture.paths,'relative paths');
      for (const name of fixture.paths) {
        const ratio=record.relativeTo[baseline][name];
        exactKeys(ratio,['speedup','cyclesSavedPercent'],'relative metrics');
        equalNumber(ratio.speedup,medians[baseline]/medians[name],'speedup '+label+' '+baseline+'/'+name);
        equalNumber(ratio.cyclesSavedPercent,(1-medians[name]/medians[baseline])*100,
          'saved percent '+label+' '+baseline+'/'+name);
        requireThat(ratio.speedup>0,'Speedup ratio must be positive '+label);
      }
    }
    const words=7*record.outputWords, compared=words*(fixture.paths.length-1);
    requireThat(record.checkedOutputWords===words && record.checkedComparisons===compared,
      'Wrong exact comparison counts '+label);
    checked+=words; comparisons+=compared;
  }
  requireThat(cases.every(fixture=>seen.has(fixture.id)),'Incomplete workload coverage');
  safeInteger(data.checkedOutputWords,1,'checkedOutputWords');
  safeInteger(data.checkedComparisons,1,'checkedComparisons');
  requireThat(data.checkedOutputWords===checked && data.checkedComparisons===comparisons,
    'Total native comparison counts disagree');
  return data;
}
if (require.main===module) {
  try {
    requireThat(process.argv.length<=3,'Usage: node validate.cjs [results.json|-]');
    const input=process.argv[2]||path.join(__dirname,'results.json');
    const data=validate(JSON.parse(fs.readFileSync(input==='-'?0:input,'utf8')));
    console.log(`Validated ${data.records.length} native SH-2 workload cases; ${data.checkedOutputWords} output words; `+
      `${data.checkedComparisons} exact path comparisons. Setup included; slower and small cases retained. No hardware or browser timing.`);
  } catch (error) {
    console.error('Workload model validation rejected: '+error.message); process.exitCode=1;
  }
}
module.exports={validate,validateManifest,KERNEL_SET,BASELINE_COMMIT,CORE_COMMIT,BASELINE_FILES,
  COST_KEYS,META_KEYS,HASH_KEYS,SPECS,LIMITATIONS,INCLUDES,EXCLUDES,REQUIRED_LIBRARY_SOURCES,baselinePaths};
