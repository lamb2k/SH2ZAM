/* Validate isolated SH-2 interpreter evidence. MIT; SH2ZAM Contributors.
 * These are guest instruction-model cycles, never physical-console timings. */
const fs=require('node:fs'),path=require('node:path');
const KERNEL_SET='float-safe-prepared-fused-sh2-model-v1';
const PATHS=['reference','safe','prepared','mac','dmuls'],SAMPLES=21;
const median=values=>[...values].sort((a,b)=>a-b)[Math.floor(values.length/2)];
function validate(data,manifest){
  const fail=message=>{throw new Error(message);};
  if(data?.schema!==1||data.kernelSet!==KERNEL_SET||data.fixtureSHA256!==manifest.fixtureSHA256)fail('Wrong SH-2 model schema, kernels or fixtures');
  const run=data.run;
  if(run?.environment!=='isolated-sh2-interpreter'||run.memoryModel!=='flat RAM')fail('An isolated instruction model is required; never console timing');
  for(const key of ['sourceCommit','interpreterCommit'])if(!/^[a-f0-9]{40}$/.test(run[key]))fail('Missing source identity '+key);
  for(const key of ['elfSHA256','interpreterSourceSHA256'])if(!/^[a-f0-9]{64}$/.test(run[key]))fail('Missing binary/core hash '+key);
  for(const key of ['evidenceUrl','interpreterRepository'])if(typeof run[key]!=='string'||!run[key].startsWith('https://github.com/'))fail('Missing public provenance '+key);
  if(!Array.isArray(run.limitations)||run.limitations.length<3||run.limitations.some(value=>typeof value!=='string'||!value.trim()))fail('Model limitations must be explicit');
  const guards=data.guards;
  if(guards?.saturationMode?.candidateMask!==5||guards.saturationMode.macInstructions!==0||guards.saturationMode.fixedMismatches!==0||guards.saturationMode.srSRetained!==true)fail('SR.S fallback guard did not pass');
  if(guards?.fullRange?.candidateMask!==0||guards.fullRange.macInstructions!==0||guards.fullRange.fixedMismatches!==0)fail('Full-range fallback guard did not pass');
  if(!Array.isArray(data.records)||data.records.length!==manifest.cases.length)fail('All shared workloads are required');
  const seen=new Set();
  const samples=(values,label)=>{
    if(!Array.isArray(values)||values.length!==SAMPLES||values.some(value=>!Number.isSafeInteger(value)||value<1))fail('Invalid guest samples '+label);
    return values;
  };
  const records=data.records.map(record=>{
    const key=record.scene+':'+record.detail,fixture=manifest.cases.find(row=>row.scene===record.scene&&row.detail===record.detail);
    if(!fixture||seen.has(key)||record.vertices!==fixture.vertices||record.sampleCount!==SAMPLES)fail('Wrong/duplicate model workload '+key);
    seen.add(key);
    if(record.fixedMismatches!==0||!Number.isFinite(record.maximumWorldError)||record.maximumWorldError<0||record.maximumWorldError>2/65536)fail('Model precision did not pass');
    if(!Array.isArray(record.candidateMasks)||record.candidateMasks.length!==SAMPLES||record.candidateMasks.some(mask=>mask!==7))fail('This model dataset must execute every native candidate');
    const setup=samples(record.planPreparationCycles,'plan cycles'),setupInstructions=samples(record.planPreparationInstructions,'plan instructions');
    const cycles={},kernelCycles={},instructions={};
    for(const name of PATHS){
      const values=samples(record.kernelCycles?.[name],name+' cycles');
      const counts=samples(record.kernelInstructions?.[name],name+' instructions');
      const checks=record.outputChecksums?.[name];
      if(!Array.isArray(checks)||checks.length!==SAMPLES||checks.some(value=>!/^[a-f0-9]{8}$/.test(value)))fail('Invalid output checksum '+name);
      if(name!=='reference'&&checks.some((value,i)=>value!==record.outputChecksums.safe[i]))fail('Native output checksums disagree');
      const includesPlan=['prepared','mac','dmuls'].includes(name);
      cycles[name]=median(values.map((value,i)=>value+(includesPlan?setup[i]:0)));
      instructions[name]=median(counts.map((value,i)=>value+(includesPlan?setupInstructions[i]:0)));
      kernelCycles[name]=median(values);
      if(record.medianCycles?.[name]!==cycles[name]||record.medianKernelCycles?.[name]!==kernelCycles[name]||record.medianInstructions?.[name]!==instructions[name])fail('Stored median omits setup or disagrees with raw samples '+name);
    }
    const referenceOver={};
    for(const name of PATHS.slice(1)){
      referenceOver[name]=cycles.reference/cycles[name];
      if(Math.abs(record.referenceOver?.[name]-referenceOver[name])>1e-10||!Number.isFinite(record.referenceOver?.[name]))fail('Stored ratio disagrees with raw samples');
    }
    return {scene:record.scene,detail:record.detail,vertices:record.vertices,sampleCount:SAMPLES,
      medianCycles:cycles,medianKernelCycles:kernelCycles,medianInstructions:instructions,referenceOver,
      fixedMismatches:0,maximumWorldError:record.maximumWorldError};
  });
  return {schema:1,kernelSet:KERNEL_SET,fixtureSHA256:data.fixtureSHA256,run,guards,records};
}
if(require.main===module){
  const root=path.resolve(__dirname,'../..');
  const input=process.argv[2]||path.join(__dirname,'sh2-model/results.json');
  const data=validate(JSON.parse(fs.readFileSync(input)),JSON.parse(fs.readFileSync(path.join(__dirname,'manifest.json'))));
  fs.writeFileSync(path.join(root,'exhibits/sh2-model-results.js'),'// Generated from validated isolated SH-2 instruction-model evidence.\nwindow.SH2ZAM_MODEL_RESULTS='+JSON.stringify(data)+';\n');
  console.log('Validated isolated SH-2 model workloads:',data.records.length);
}
module.exports={validate,KERNEL_SET,PATHS};
