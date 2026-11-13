/* Validate five-kernel captures and generate the browser's dataset. MIT. */
const fs=require('node:fs'),path=require('node:path');
const SCHEMA=3,KERNEL_SET='float-safe-prepared-fused-v1',SAMPLES=21,MAX_ERROR_LSB=66,PREPARE_ITERATIONS=64;
function validate(data,manifest){
  const fail=m=>{throw new Error(m);};
  if(!data||data.schema!==SCHEMA||data.fixtureSHA256!==manifest.fixtureSHA256)fail('Schema or fixture mismatch; five-kernel prepared capture required');
  if(!Array.isArray(data.records))fail('records must be an array');
  if(data.run===null){if(data.records.length)fail('Records need provenance');return {schema:SCHEMA,fixtureSHA256:data.fixtureSHA256,run:null,records:[]};}
  const r=data.run;
  if(!r||!['physical-console','emulator'].includes(r.environment))fail('Explicit capture environment required');
  if(r.kernelSet!==KERNEL_SET||r.backend!=='sh2-dmuls'||r.timerDivisor!==128||r.chunkVertices!==32||r.prepareIterations!==PREPARE_ITERATIONS)fail('Wrong kernels/backend/timer/workload/preparation count');
  if(!Number.isFinite(r.clockHz)||r.clockHz<26000000||r.clockHz>30000000)fail('Validated Saturn CPU clock in Hz required');
  if(!/^[a-f0-9]{40}$/.test(r.sourceCommit))fail('Full source commit required');
  if(!/^[a-f0-9]{64}$/.test(r.imageSHA256))fail('Boot image SHA256 required');
  for(const field of ['compiler','flags','console','videoMode','cachePolicy','memoryPlacement','capturedAt'])if(typeof r[field]!=='string'||!r[field].trim())fail('Missing '+field);
  if(!Number.isFinite(Date.parse(r.capturedAt)))fail('Invalid capture date');
  if(typeof r.evidenceUrl!=='string'||!/^https:\/\//.test(r.evidenceUrl))fail('HTTPS evidence URL required');
  const seen=new Set(),median=xs=>[...xs].sort((a,b)=>a-b)[Math.floor(SAMPLES/2)];
  const samples=(row,field,limit)=>{
    const values=row[field];
    if(!Array.isArray(values)||values.length!==SAMPLES||values.some(x=>!Number.isSafeInteger(x)||x<0||x>limit))fail('Invalid samples '+field);
    return values;
  };
  const records=data.records.map(row=>{
    const key=row.scene+':'+row.detail,c=manifest.cases.find(c=>c.scene===row.scene&&c.detail===row.detail);
    if(!c||seen.has(key)||row.vertices!==c.vertices)fail('Invalid/duplicate workload '+key);seen.add(key);
    if(row.invalid!==false||!Number.isInteger(row.maxErrorLSB)||row.maxErrorLSB<0||row.maxErrorLSB>MAX_ERROR_LSB)fail('Invalid capture or precision bound');
    if(row.fixedMismatches!==0)fail('All four fixed-point outputs must match exactly');
    const checksumFields=['checksum','referenceChecksum','generalChecksum','optimizedChecksum','macChecksum','dmulsChecksum'];
    for(const field of checksumFields)if(typeof row[field]!=='string'||!/^[a-f0-9]{8}$/.test(row[field]))fail('Invalid '+field);
    for(const field of ['optimizedChecksum','macChecksum','dmulsChecksum'])if(row[field]!==row.generalChecksum)fail('Fixed-point checksum mismatch');
    const maximumKernelTicks=Math.ceil(c.vertices/r.chunkVertices)*65535;
    for(const field of ['referenceTicks','generalTicks','optimizedTicks','macTicks','dmulsTicks','overheadTicks'])samples(row,field,maximumKernelTicks);
    samples(row,'prepareTicks',65535);samples(row,'prepareOverheadTicks',65535);
    const masks=samples(row,'candidateMasks',7);
    if(masks.some(mask=>(mask&6)!==0&&(mask&1)===0))fail('Native candidate requires the prepared bound proof');
    const preparedAvailable=masks.every(mask=>(mask&1)!==0);
    const macAvailable=masks.every(mask=>(mask&2)!==0);
    const dmulsAvailable=masks.every(mask=>(mask&4)!==0);
    const setup=row.prepareTicks.map((ticks,i)=>(ticks-row.prepareOverheadTicks[i])/r.prepareIterations);
    if(setup.some(x=>x<0))fail('Negative preparation time after overhead');
    const adjusted=field=>row[field].map((ticks,i)=>ticks-row.overheadTicks[i]);
    const ref=adjusted('referenceTicks'),general=adjusted('generalTicks');
    const optimizedKernel=adjusted('optimizedTicks'),macKernel=adjusted('macTicks'),dmulsKernel=adjusted('dmulsTicks');
    if([...ref,...general,...optimizedKernel,...macKernel,...dmulsKernel].some(x=>x<=0))fail('Below timer resolution after overhead');
    const inclusive=kernel=>kernel.map((ticks,i)=>ticks+setup[i]);
    const cycles=values=>median(values)*r.timerDivisor;
    const referenceCycles=cycles(ref),generalCycles=cycles(general);
    const prepareCycles=cycles(setup),optimizedKernelCycles=cycles(optimizedKernel),optimizedCycles=cycles(inclusive(optimizedKernel));
    // Preserve raw candidate samples while withholding candidate performance
    // summaries if even one pose used a fallback. No fallback is labeled active.
    const macKernelCycles=macAvailable?cycles(macKernel):null,macCycles=macAvailable?cycles(inclusive(macKernel)):null;
    const dmulsKernelCycles=dmulsAvailable?cycles(dmulsKernel):null,dmulsCycles=dmulsAvailable?cycles(inclusive(dmulsKernel)):null;
    const ms=value=>value===null?null:value/r.clockHz*1000;
    const ratio=(numerator,denominator)=>denominator===null?null:numerator/denominator;
    return {...row,preparedAvailable,macAvailable,dmulsAvailable,
      referenceCycles,generalCycles,prepareCycles,optimizedKernelCycles,optimizedCycles,
      macKernelCycles,macCycles,dmulsKernelCycles,dmulsCycles,
      referenceMs:ms(referenceCycles),generalMs:ms(generalCycles),prepareMs:ms(prepareCycles),
      optimizedKernelMs:ms(optimizedKernelCycles),optimizedMs:ms(optimizedCycles),
      macKernelMs:ms(macKernelCycles),macMs:ms(macCycles),dmulsKernelMs:ms(dmulsKernelCycles),dmulsMs:ms(dmulsCycles),
      referenceOverGeneral:referenceCycles/generalCycles,referenceOverOptimized:referenceCycles/optimizedCycles,generalOverOptimized:generalCycles/optimizedCycles,
      referenceOverMac:ratio(referenceCycles,macCycles),generalOverMac:ratio(generalCycles,macCycles),optimizedOverMac:ratio(optimizedCycles,macCycles),
      referenceOverDmuls:ratio(referenceCycles,dmulsCycles),generalOverDmuls:ratio(generalCycles,dmulsCycles),optimizedOverDmuls:ratio(optimizedCycles,dmulsCycles)};
  });
  return {schema:SCHEMA,fixtureSHA256:data.fixtureSHA256,run:r,records};
}
if(require.main===module){
  const root=path.resolve(__dirname,'../..'),input=process.argv[2]||path.join(__dirname,'results/captures.json');
  const data=validate(JSON.parse(fs.readFileSync(input)),JSON.parse(fs.readFileSync(path.join(__dirname,'manifest.json'))));
  fs.writeFileSync(path.join(root,'exhibits/saturn-results.js'),'// Generated from validated captures.\nwindow.SH2ZAM_SATURN_RESULTS='+JSON.stringify(data)+';\n');
  console.log('Validated recorded results:',data.records.length);
}
module.exports={validate,SCHEMA,KERNEL_SET,MAX_ERROR_LSB,PREPARE_ITERATIONS};
