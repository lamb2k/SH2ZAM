/* Validate three-kernel captures and generate the browser's dataset. MIT. */
const fs=require('node:fs'),path=require('node:path');
const SCHEMA=2,KERNEL_SET='float-general-batch-v1',SAMPLES=21,MAX_ERROR_LSB=66;
function validate(data,manifest){
  const fail=m=>{throw new Error(m);};
  if(!data||data.schema!==SCHEMA||data.fixtureSHA256!==manifest.fixtureSHA256)fail('Schema or fixture mismatch; three-kernel capture required');
  if(!Array.isArray(data.records))fail('records must be an array');
  if(data.run===null){if(data.records.length)fail('Records need provenance');return {schema:SCHEMA,fixtureSHA256:data.fixtureSHA256,run:null,records:[]};}
  const r=data.run;
  if(!r||!['physical-console','emulator'].includes(r.environment))fail('Explicit capture environment required');
  if(r.kernelSet!==KERNEL_SET||r.backend!=='sh2-dmuls'||r.timerDivisor!==128||r.chunkVertices!==32)fail('Wrong kernels/backend/timer/workload');
  if(!Number.isFinite(r.clockHz)||r.clockHz<26000000||r.clockHz>30000000)fail('Validated Saturn CPU clock in Hz required');
  if(!/^[a-f0-9]{40}$/.test(r.sourceCommit))fail('Full source commit required');
  if(!/^[a-f0-9]{64}$/.test(r.imageSHA256))fail('Boot image SHA256 required');
  for(const field of ['compiler','flags','console','videoMode','cachePolicy','memoryPlacement','capturedAt'])if(typeof r[field]!=='string'||!r[field].trim())fail('Missing '+field);
  if(!Number.isFinite(Date.parse(r.capturedAt)))fail('Invalid capture date');
  if(typeof r.evidenceUrl!=='string'||!/^https:\/\//.test(r.evidenceUrl))fail('HTTPS evidence URL required');
  const seen=new Set(),median=xs=>[...xs].sort((a,b)=>a-b)[Math.floor(SAMPLES/2)];
  const records=data.records.map(row=>{
    const key=row.scene+':'+row.detail,c=manifest.cases.find(c=>c.scene===row.scene&&c.detail===row.detail);
    if(!c||seen.has(key)||row.vertices!==c.vertices)fail('Invalid/duplicate workload '+key);seen.add(key);
    if(row.invalid!==false||!Number.isInteger(row.maxErrorLSB)||row.maxErrorLSB<0||row.maxErrorLSB>MAX_ERROR_LSB)fail('Invalid capture or precision bound');
    if(row.fixedMismatches!==0)fail('General and optimized outputs must match exactly');
    for(const field of ['checksum','referenceChecksum','generalChecksum','optimizedChecksum'])if(typeof row[field]!=='string'||!/^[a-f0-9]{8}$/.test(row[field]))fail('Invalid '+field);
    if(row.generalChecksum!==row.optimizedChecksum)fail('Fixed-point checksum mismatch');
    for(const field of ['referenceTicks','generalTicks','optimizedTicks','overheadTicks'])if(!Array.isArray(row[field])||row[field].length!==SAMPLES||row[field].some(x=>!Number.isSafeInteger(x)||x<0||x>Math.ceil(c.vertices/32)*65535))fail('Invalid samples '+field);
    const ref=row.referenceTicks.map((x,i)=>x-row.overheadTicks[i]);
    const general=row.generalTicks.map((x,i)=>x-row.overheadTicks[i]);
    const optimized=row.optimizedTicks.map((x,i)=>x-row.overheadTicks[i]);
    if([...ref,...general,...optimized].some(x=>x<=0))fail('Below timer resolution after overhead');
    const referenceCycles=median(ref)*128,generalCycles=median(general)*128,optimizedCycles=median(optimized)*128;
    return {...row,referenceCycles,generalCycles,optimizedCycles,
      referenceMs:referenceCycles/r.clockHz*1000,generalMs:generalCycles/r.clockHz*1000,optimizedMs:optimizedCycles/r.clockHz*1000,
      referenceOverGeneral:referenceCycles/generalCycles,referenceOverOptimized:referenceCycles/optimizedCycles,generalOverOptimized:generalCycles/optimizedCycles};
  });
  return {schema:SCHEMA,fixtureSHA256:data.fixtureSHA256,run:r,records};
}
if(require.main===module){
  const root=path.resolve(__dirname,'../..'),input=process.argv[2]||path.join(__dirname,'results/captures.json');
  const data=validate(JSON.parse(fs.readFileSync(input)),JSON.parse(fs.readFileSync(path.join(__dirname,'manifest.json'))));
  fs.writeFileSync(path.join(root,'exhibits/saturn-results.js'),'// Generated from validated captures.\nwindow.SH2ZAM_SATURN_RESULTS='+JSON.stringify(data)+';\n');
  console.log('Validated recorded results:',data.records.length);
}
module.exports={validate,SCHEMA,KERNEL_SET,MAX_ERROR_LSB};
