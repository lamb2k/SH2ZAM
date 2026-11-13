/* Validate captured data and generate the browser's read-only dataset. MIT. */
const fs=require('node:fs'),path=require('node:path');
function validate(data,manifest){
  const fail=m=>{throw new Error(m);};
  if(data.schema!==1||data.fixtureSHA256!==manifest.fixtureSHA256)fail('Schema or fixture mismatch');
  if(!Array.isArray(data.records))fail('records must be an array');
  if(data.run===null){if(data.records.length)fail('Records need provenance');return {schema:1,run:null,records:[]};}
  const r=data.run;
  if(!r||!['physical-console','emulator'].includes(r.environment))fail('Explicit capture environment required');
  if(r.backend!=='sh2-dmuls'||r.timerDivisor!==128||r.chunkVertices!==32)fail('Wrong backend/timer/workload');
  if(!Number.isFinite(r.clockHz)||r.clockHz<26000000||r.clockHz>30000000)fail('Validated Saturn CPU clock in Hz required');
  if(!/^[a-f0-9]{40}$/.test(r.sourceCommit))fail('Full source commit required');
  if(!/^[a-f0-9]{64}$/.test(r.imageSHA256))fail('Boot image SHA256 required');
  for(const field of ['compiler','flags','console','videoMode','cachePolicy','memoryPlacement','capturedAt'])if(typeof r[field]!=='string'||!r[field].trim())fail('Missing '+field);
  if(!Number.isFinite(Date.parse(r.capturedAt)))fail('Invalid capture date');
  if(typeof r.evidenceUrl!=='string'||!/^https:\/\//.test(r.evidenceUrl))fail('HTTPS evidence URL required');
  const seen=new Set(),median=xs=>[...xs].sort((a,b)=>a-b)[10];
  const records=data.records.map(row=>{
    const key=row.scene+':'+row.detail,c=manifest.cases.find(c=>c.scene===row.scene&&c.detail===row.detail);
    if(!c||seen.has(key)||row.vertices!==c.vertices)fail('Invalid/duplicate workload '+key);seen.add(key);
    if(row.invalid!==false||!Number.isInteger(row.maxErrorLSB)||row.maxErrorLSB<0||!/^[a-f0-9]{8}$/.test(row.checksum))fail('Invalid capture/error/checksum');
    for(const field of ['referenceTicks','fixedTicks','overheadTicks'])if(!Array.isArray(row[field])||row[field].length!==21||row[field].some(x=>!Number.isSafeInteger(x)||x<0||x>Math.ceil(c.vertices/32)*65535))fail('Invalid samples '+field);
    const ref=row.referenceTicks.map((x,i)=>x-row.overheadTicks[i]),fixed=row.fixedTicks.map((x,i)=>x-row.overheadTicks[i]);
    if([...ref,...fixed].some(x=>x<=0))fail('Below timer resolution after overhead');
    const referenceCycles=median(ref)*128,fixedCycles=median(fixed)*128;
    return {...row,referenceCycles,fixedCycles,referenceMs:referenceCycles/r.clockHz*1000,fixedMs:fixedCycles/r.clockHz*1000,referenceOverFixed:referenceCycles/fixedCycles};
  });
  return {schema:1,fixtureSHA256:data.fixtureSHA256,run:r,records};
}
if(require.main===module){
  const root=path.resolve(__dirname,'../..'),input=process.argv[2]||path.join(__dirname,'results/captures.json');
  const data=validate(JSON.parse(fs.readFileSync(input)),JSON.parse(fs.readFileSync(path.join(__dirname,'manifest.json'))));
  fs.writeFileSync(path.join(root,'exhibits/saturn-results.js'),'// Generated from validated captures.\nwindow.SH2ZAM_SATURN_RESULTS='+JSON.stringify(data)+';\n');
  console.log('Validated recorded results:',data.records.length);
}
module.exports={validate};
