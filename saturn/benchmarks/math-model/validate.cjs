/* Strict publication gate for native old/new SH-2 math evidence.
 * MIT; Copyright (c) 2026 SH2ZAM Contributors. */
'use strict';
const fs=require('node:fs'),path=require('node:path'),crypto=require('node:crypto');
const KERNEL_SET='previous-optimized-math-sh2-model-v1';
const BASELINE_COMMIT='10786b64f4593d647d06d2a4a73e752c0c413dd0';
const CORE_COMMIT='7e38821dbac265490f115e163c523a939acda759';
const PROFILES=['bounded','tiny','full-range'];
const BASELINE_FILES=['sh2zam.c','sh2zam_angles.c','sh2zam_trig.c','sh2zam_transform.c','sh2zam_geometry.c','sh2zam_internal.h'];
const COST_KEYS=['cycles','instructions','dataReads','dataWrites','macInstructions','dmulsInstructions'];
const FLAGS=['-m2','-mb','-O2','-fno-fast-math','-ffp-contract=off','-fno-lto'];
const LIMITATIONS=[
  'Not physical Saturn timing or full-system emulator timing',
  'No cache or bus wait-state model',
  'No pipeline or multiplier-contention model',
  'No DMA, interrupts, timers or peripheral integration',
  'One SH-2 CPU; no second-CPU scheduling or shared-memory contention',
  'No conversion to milliseconds, frame rate or clock-frequency estimates'
];
const HASH_KEYS=['elfSHA256','symbolSHA256','rawCaptureSHA256','interpreterSourceSHA256',
  'bridgeSourceSHA256','workloadsSourceSHA256','casesSourceSHA256','suiteSourceSHA256',
  'harnessSourceSHA256','captureSourceSHA256','collectorSourceSHA256','validatorSourceSHA256','baselineRenameSHA256'];
const CASES=[...fs.readFileSync(path.join(__dirname,'cases.inc'),'utf8').matchAll(/\{"([a-z0-9_]+)",(\d+)u\}/g)]
  .map(match=>({operation:match[1],outputWords:Number(match[2])}));
const median=values=>[...values].sort((a,b)=>a-b)[Math.floor(values.length/2)];
const sha=bytes=>crypto.createHash('sha256').update(bytes).digest('hex');
const fail=message=>{throw new Error(message);};
function requireThat(condition,message){if(!condition)fail(message);}
function exactKeys(value,keys,label){
  requireThat(value!==null&&typeof value==='object'&&!Array.isArray(value)&&
    Object.keys(value).length===keys.length&&keys.every(key=>Object.hasOwn(value,key)),
    'Unexpected/missing '+label+' fields');
}
function safeInteger(value,minimum,label){
  requireThat(Number.isSafeInteger(value)&&value>=minimum,'Invalid '+label+' count');
}
function hash(value,label){requireThat(typeof value==='string'&&/^[a-f0-9]{64}$/.test(value),'Missing/invalid '+label+' hash');}
function equalNumber(actual,expected,label){
  requireThat(typeof actual==='number'&&Number.isFinite(actual)&&
    Math.abs(actual-expected)<=1e-11*Math.max(1,Math.abs(expected)),'Derived metric differs: '+label);
}
function strings(value,label){
  requireThat(Array.isArray(value)&&value.length>0&&value.every(v=>typeof v==='string'&&v.trim()),'Missing '+label);
}
function family(operation){
  if(operation.startsWith('vec'))return 'vector';
  if(operation.startsWith('quat'))return 'quaternion';
  if(operation.startsWith('mat'))return 'matrix';
  if(['sin','cos','sincos','tan','atan2','atan','asin','acos'].includes(operation))return 'trig';
  if(operation==='project_clip'||operation.startsWith('sphere_'))return 'geometry';
  return 'scalar';
}
function validate(data){
  exactKeys(data,['schema','kernelSet','comparison','baseline','run','batchSize','samplesPerProfile',
    'profiles','checkedOutputWords','mismatches','records'],'dataset');
  requireThat(data.schema===1&&data.kernelSet===KERNEL_SET,'Wrong native math model schema');
  exactKeys(data.comparison,['before','after','independentNonShazamBaseline'],'comparison');
  requireThat(data.comparison.before==='previous SH2ZAM'&&data.comparison.after==='optimized SH2ZAM'&&
    data.comparison.independentNonShazamBaseline===false,'The baseline is previous SH2ZAM, not independent plain C');
  exactKeys(data.baseline,['sourceCommit','manifestSHA256','files'],'baseline');
  requireThat(data.baseline.sourceCommit===BASELINE_COMMIT,'Wrong frozen baseline commit');
  hash(data.baseline.manifestSHA256,'baseline manifest');
  const manifestBytes=fs.readFileSync(path.join(__dirname,'legacy/manifest.json'));
  const manifest=JSON.parse(manifestBytes);
  exactKeys(manifest,['sourceCommit','files'],'baseline manifest');
  requireThat(manifest.sourceCommit===BASELINE_COMMIT&&sha(manifestBytes)===data.baseline.manifestSHA256,
    'Baseline manifest identity differs');
  exactKeys(manifest.files,BASELINE_FILES,'manifest sources');
  exactKeys(data.baseline.files,BASELINE_FILES,'baseline sources');
  for(const name of BASELINE_FILES){
    hash(data.baseline.files[name],'baseline '+name);
    requireThat(data.baseline.files[name]===manifest.files[name]&&
      sha(fs.readFileSync(path.join(__dirname,'legacy',name)))===manifest.files[name],
      'Frozen baseline bytes differ: '+name);
  }
  const run=data.run;
  exactKeys(run,['environment','sourceCommit','compiler','compileFlags','toolchainVerifiedInELF',
    'evidenceUrl','interpreterRepository','interpreterCommit','interpreterSource','interpreterModified',
    'memoryModel','cpuCount','wholeBatch','cycleUnit','hardwareMeasured','browserTimingUsed','hostResults',
    'includes','excludes','limitations','librarySourceSHA256','baselineWrapperSHA256',...HASH_KEYS],'run');
  requireThat(run.environment==='isolated-sh2-interpreter'&&run.memoryModel==='flat RAM'&&run.cpuCount===1&&
    run.wholeBatch===true&&run.cycleUnit==='upstream interpreter per-opcode modeled cycles',
    'Expected one isolated flat-RAM SH-2 instruction model');
  requireThat(run.hardwareMeasured===false&&run.browserTimingUsed===false&&run.hostResults===false,
    'This evidence contains native instruction-model results only');
  requireThat(typeof run.sourceCommit==='string'&&/^[a-f0-9]{40}$/.test(run.sourceCommit),'Missing source commit');
  requireThat(run.interpreterRepository==='https://github.com/Yabause/yabause'&&run.interpreterCommit===CORE_COMMIT&&
    run.interpreterSource==='yabause/src/sh2int.c'&&run.interpreterModified===false,'Wrong or modified interpreter');
  requireThat(typeof run.compiler==='string'&&run.compiler.trim()&&run.toolchainVerifiedInELF===true,
    'Missing ELF-verified compiler provenance');
  requireThat(typeof run.compileFlags==='string','Missing compiler flags');
  const flags=run.compileFlags.trim().split(/\s+/);
  requireThat(FLAGS.every(flag=>flags.includes(flag))&&!flags.includes('-ffast-math')&&!flags.includes('-flto'),
    'Unexpected native compile configuration');
  requireThat(typeof run.evidenceUrl==='string'&&
    /^https:\/\/github\.com\/[A-Za-z0-9_.-]+\/[A-Za-z0-9_.-]+\/actions\/(?:runs\/[0-9]+|workflows\/[^\s?#]+)$/.test(run.evidenceUrl),
    'Missing public GitHub Actions evidence');
  for(const key of HASH_KEYS)hash(run[key],key);
  exactKeys(run.librarySourceSHA256,BASELINE_FILES,'optimized library sources');
  for(const name of BASELINE_FILES)hash(run.librarySourceSHA256[name],'optimized '+name);
  const wrappers=BASELINE_FILES.filter(name=>name.endsWith('.c')).map(name=>'legacy_'+name);
  exactKeys(run.baselineWrapperSHA256,wrappers,'baseline wrapper sources');
  for(const name of wrappers)hash(run.baselineWrapperSHA256[name],'wrapper '+name);
  strings(run.includes,'measured scope'); strings(run.excludes,'excluded scope');
  requireThat(Array.isArray(run.limitations)&&run.limitations.length===LIMITATIONS.length&&
    LIMITATIONS.every((value,index)=>run.limitations[index]===value),'Missing or altered model limitations');
  requireThat(data.batchSize===32&&data.samplesPerProfile===7&&Array.isArray(data.profiles)&&
    data.profiles.length===3&&PROFILES.every((p,i)=>data.profiles[i]===p),'Wrong batch/sample/profile coverage');
  requireThat(CASES.length===48&&new Set(CASES.map(row=>row.operation)).size===48,'Wrong canonical operation list');
  requireThat(Array.isArray(data.records)&&data.records.length===144,'All 144 operation/profile records are required');
  requireThat(data.mismatches===0,'Native output mismatches must be zero');
  const expected=new Map(CASES.map(row=>[row.operation,row.outputWords])),seen=new Set();
  let checked=0;
  for(const record of data.records){
    exactKeys(record,['operation','family','profile','outputWords','samples','sampleCount','srSRetained','srSOutputMatch',
      'medianCycles','cyclesPerCall','speedup','cyclesSavedPercent','checkedOutputWords'],'record');
    const key=record.operation+':'+record.profile;
    requireThat(expected.has(record.operation)&&PROFILES.includes(record.profile)&&!seen.has(key),'Unknown/duplicate workload '+key);
    seen.add(key);
    requireThat(record.family===family(record.operation)&&record.outputWords===expected.get(record.operation),
      'Wrong operation metadata '+key);
    requireThat(record.srSRetained===true&&record.srSOutputMatch===true,'SR.S preservation/equality failed '+key);
    requireThat(record.sampleCount===7&&Array.isArray(record.samples)&&record.samples.length===7,'Incomplete samples '+key);
    for(const sample of record.samples){
      exactKeys(sample,['inputChecksum','before','after','outputChecksum','mismatches'],'sample');
      for(const name of ['inputChecksum','outputChecksum'])
        requireThat(typeof sample[name]==='string'&&/^[a-f0-9]{8}$/.test(sample[name]),'Invalid '+name+' '+key);
      requireThat(sample.mismatches===0,'Native sample output mismatch '+key);
      for(const version of ['before','after']){
        const cost=sample[version]; exactKeys(cost,COST_KEYS,'cost');
        for(const metric of COST_KEYS)safeInteger(cost[metric],['cycles','instructions'].includes(metric)?1:0,key+' '+version+' '+metric);
        requireThat(cost.macInstructions+cost.dmulsInstructions<=cost.instructions,'Impossible native counters '+key);
      }
    }
    exactKeys(record.medianCycles,['before','after'],'medianCycles');
    exactKeys(record.cyclesPerCall,['before','after'],'cyclesPerCall');
    const medians={};
    for(const version of ['before','after']){
      medians[version]=median(record.samples.map(sample=>sample[version].cycles));
      requireThat(record.medianCycles[version]===medians[version],'Median disagrees with samples '+key);
      equalNumber(record.cyclesPerCall[version],medians[version]/32,'cycles/call '+key+' '+version);
    }
    equalNumber(record.speedup,medians.before/medians.after,'speedup '+key);
    equalNumber(record.cyclesSavedPercent,(1-medians.after/medians.before)*100,'saved percent '+key);
    requireThat(record.speedup>0,'Speedup must be positive '+key);
    const words=32*7*record.outputWords;
    requireThat(record.checkedOutputWords===words,'Wrong checked output count '+key);
    checked+=words;
  }
  for(const row of CASES)for(const profile of PROFILES)
    requireThat(seen.has(row.operation+':'+profile),'Missing workload '+row.operation+':'+profile);
  safeInteger(data.checkedOutputWords,1,'checkedOutputWords');
  requireThat(data.checkedOutputWords===checked,'Total native check count disagrees');
  return data;
}

if(require.main===module){
  try{
    const input=process.argv[2]||path.join(__dirname,'results.json');
    requireThat(process.argv.length<=3,'Usage: node validate.cjs [results.json|-]');
    const data=validate(JSON.parse(fs.readFileSync(input==='-'?0:input,'utf8')));
    const wins=data.records.filter(record=>record.speedup>1).length;
    const losses=data.records.filter(record=>record.speedup<1).length;
    console.log(`Validated native SH-2 math model: ${data.records.length} workloads; ${data.checkedOutputWords} exact output words; `+
      `${wins} faster, ${losses} slower, ${data.records.length-wins-losses} unchanged. No hardware or browser timing.`);
  }catch(error){console.error('Math model validation rejected: '+error.message);process.exitCode=1;}
}
module.exports={validate,KERNEL_SET,BASELINE_COMMIT,CORE_COMMIT,PROFILES,CASES,LIMITATIONS,COST_KEYS,HASH_KEYS,family};
