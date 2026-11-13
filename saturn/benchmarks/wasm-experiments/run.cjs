/* Private exact-integer WASM candidate comparison; Node/V8, not SH-2 timing. */
const fs=require('node:fs');
const vm=require('node:vm');
const path=require('node:path');
const crypto=require('node:crypto');
const os=require('node:os');
const assert=require('node:assert/strict');
const {performance}=require('node:perf_hooks');
const root=path.resolve(__dirname,'../../..');
const wasmPath=path.resolve(process.argv[2]||path.join(root,'build/wasm-experiments/math.wasm'));
const minimumMs=Number(process.env.SAMPLE_MS||10);
assert.ok(Number.isFinite(minimumMs)&&minimumMs>=5,'SAMPLE_MS must be >=5');
const details=(process.env.DETAILS||'4').split(',').map(Number);
assert.ok(details.every(d=>Number.isInteger(d)&&d>=1&&d<=5));
const paths=['before','after','prepared','prepared_inclusive','wide','packed','rows','split','narrow'];
const fixedPaths=paths.filter(p=>p!=='before');
const labels={before:'Float reference (native WASM float)',after:'Current safe fixed batch',
 prepared:'Prepared portable kernel',prepared_inclusive:'Plan preparation + prepared portable kernel',
 wide:'Wide SIMD, AoS input',packed:'Wide SIMD, prepacked input',rows:'Wide SIMD, rows per vertex',
 split:'Signed16 split dot, prepacked input',narrow:'Signed16 dot with residual bound, prepacked input'};
const ctx={window:{}};vm.createContext(ctx);
vm.runInContext(fs.readFileSync(path.join(root,'exhibits/meshes.js'),'utf8'),ctx);
const meshes=ctx.SH2ZAM_MESHES;
const hash=(bytes)=>crypto.createHash('sha256').update(bytes).digest('hex');
const measure=(fn)=>{
 let iterations=64,elapsed=0;
 do {
  const start=performance.now();for(let i=0;i<iterations;i++)fn();
  elapsed=performance.now()-start;
  if(elapsed<minimumMs)iterations*=2;
 } while(elapsed<minimumMs);
 return {ms:elapsed/iterations,iterations,elapsed};
};
const summary=(samples)=>{
 const values=samples.map(s=>s.ms).sort((a,b)=>a-b);
 const quantile=(p)=>values[Math.floor((values.length-1)*p)];
 return {median:quantile(.5),p10:quantile(.1),p90:quantile(.9),min:values[0],max:values.at(-1)};
};

function exactRow(input,matrix,i,row) {
 const o=3*i,r=4*row;
 let sum=BigInt(input[o])*BigInt(matrix[r])+BigInt(input[o+1])*BigInt(matrix[r+1])
       +BigInt(input[o+2])*BigInt(matrix[r+2]);
 sum=(sum<0n?(sum-65535n)/65536n:sum/65536n)+BigInt(matrix[r+3]);
 return Number(sum>2147483647n?2147483647n:sum< -2147483648n?-2147483648n:sum);
}

(async()=>{
 const bytes=fs.readFileSync(wasmPath);
 const w=(await WebAssembly.instantiate(bytes,{})).instance.exports;
 const allInput=new Int32Array(w.memory.buffer,w.input_ptr(),3*w.capacity());
 const matrix=new Int32Array(w.memory.buffer,w.matrix_ptr(),12);
 const views=Object.fromEntries(fixedPaths.map(p=>[p,new Int32Array(w.memory.buffer,w[p+'_ptr'](),3*w.capacity())]));
 const general=new Int32Array(w.memory.buffer,w.general_ptr(),3*w.capacity());
 const floats=new Float32Array(w.memory.buffer,w.before_ptr(),3*w.capacity());
 let checksum=0,checkedComponents=0;
 const eligibility={prepared:0,split:0,narrow:0,fallback:0};
 const check=(n,independent=false)=>{
  w.general();for(const p of paths){
   w[p]();
   if(p!=='before')for(let i=0;i<3*n;i++)assert.equal(views[p][i],general[i],p+' component '+i);
  }
  for(let i=0;i<3*n;i++){
   if(independent)assert.equal(general[i],exactRow(allInput,matrix,Math.floor(i/3),i%3),'BigInt oracle');
   checksum=(checksum^general[i])>>>0;checkedComponents++;
  }
  eligibility.prepared+=w.fast_path();eligibility.split+=w.split_ok();
  eligibility.narrow+=w.narrow_ok();eligibility.fallback+=!w.fast_path();
 };
 let state=0x25e31a4d;
 const random=()=>{state^=state<<13;state^=state>>>17;state^=state<<5;return state>>>0;};
 const lengths=[0,1,2,3,4,5,7,8,9,31,32,33];
 const extremes=[-2147483648,2147483647,-2147450881,2147450880,-65536,-32768,-1,0,1,32767,32768,65536];
 let oracleCases=0;
 for(let round=0;round<8;round++)for(const n of lengths){
  for(let i=0;i<3*n;i++)allInput[i]=round<4?(random()%1048577)-524288:extremes[random()%extremes.length];
  assert.equal(w.prepare(n),1);
  for(let r=0;r<3;r++)for(let c=0;c<4;c++)matrix[4*r+c]=round<4?(random()%262145)-131072:extremes[random()%extremes.length];
  assert.equal(w.apply_matrix(),1);check(n,true);oracleCases++;
 }
 // Exercise signed16 residual pair overflow, positive rounded-high overflow,
 // fractional negative sums, exact negative floor, and vector tails directly.
 const adversarial=[
  {input:[-32768,-32768,0],row:[-32768,-32768,0,0]},
  {input:[2147450880,1,0],row:[1,0,0,0]},
  {input:[1,0,0],row:[2147450880,0,0,0]},
  {input:[-32768,-32768,-32768],row:[32767,32767,32767,-1]},
  {input:[1,1,1],row:[-1,-1,-1,0]},
  {input:[65536,0,0],row:[2147483647,0,0,0]},
  {input:[65536,0,0],row:[-2147483648,0,0,0]},
 ];
 for(const fixture of adversarial){
  const n=9;for(let i=0;i<n;i++)allInput.set(fixture.input,3*i);
  assert.equal(w.prepare(n),1);for(let r=0;r<3;r++)matrix.set(fixture.row,4*r);
  assert.equal(w.apply_matrix(),1);check(n,true);oracleCases++;
 }
 assert.ok(eligibility.prepared&&eligibility.split&&eligibility.narrow&&eligibility.fallback,
           'Correctness fixtures must exercise fast and fallback paths');
 const result={schemaVersion:1,createdAt:new Date().toISOString(),
  environment:'Node/V8 WASM SIMD on this host; not a browser or Saturn measurement',
  engine:process.version+' / V8 '+process.versions.v8,
  compiler:process.env.COMPILER_ID||'Not recorded; set COMPILER_ID when running',
  host:{architecture:process.arch,platform:process.platform,cpuModel:os.cpus()[0]?.model,
        logicalCpus:os.cpus().length},
  flags:'-O3 -msimd128 -fno-fast-math -ffp-contract=off for every translation unit',
  minimumSampleMs:minimumMs,posesPerCase:21,details,labels,
  wasmSha256:hash(bytes),sourceSha256:Object.fromEntries(
   ['bridge.c','kernels.h','build.sh','run.cjs'].map(p=>[p,hash(fs.readFileSync(path.join(__dirname,p)))])),
  dependencySha256:Object.fromEntries(['exhibits/meshes.js','saturn/include/sh2zam/sh2zam.h',
   'saturn/src/sh2zam.c','saturn/src/sh2zam_trig.c','saturn/src/sh2zam_angles.c',
   'saturn/src/sh2zam_transform.c','saturn/src/sh2zam_prepared.c','saturn/src/sh2zam_geometry.c',
   'saturn/src/sh2zam_internal.h','saturn/src/sh2zam_prepared_internal.h']
   .map(p=>[p,hash(fs.readFileSync(path.join(root,p)))])),
  correctness:{independentBigIntCases:oracleCases,eligibility},runs:[]};
 for(const detail of details)for(const scene of ['saturn','cubes','torus']){
  const mesh=meshes.makeMesh(scene,detail),n=mesh.vertices.length/3;
  assert.ok(n<=w.capacity());allInput.set(mesh.vertices.map(v=>Math.round(v*65536)));
  assert.equal(w.prepare(n),1);
  const oneTime={};
  for(const p of ['prepare_float','prepare_view','pack_wide','pack_split'])
   oneTime[p]=summary(Array.from({length:7},()=>measure(w[p])));
  const samples=Object.fromEntries(paths.map(p=>[p,[]]));
  const planSamples=[],floatMatrixSamples=[],poseEligibility=[];
  let maxFloatError=0;
  for(let s=0;s<21;s++){
   const ax=(5200+Math.round(3000*Math.sin(s/21*Math.PI*2)))&65535;
   const ay=Math.floor(s/21*65536)&65535;
   assert.equal(w.setup(ax,ay),1);
   poseEligibility.push({ax,ay,prepared:!!w.fast_path(),split:!!w.split_ok(),narrow:!!w.narrow_ok()});
   for(let warm=0;warm<128;warm++)for(const p of paths)w[p]();
   // Rotate path order at every pose. Each retained sample takes >=SAMPLE_MS;
   // a shorter calibration attempt is never reported as a timing sample.
   for(let o=0;o<paths.length;o++){
    const p=paths[(s+o)%paths.length];samples[p].push(measure(w[p]));
   }
   planSamples.push(measure(w.prepare_plan));
   floatMatrixSamples.push(measure(w.prepare_float_matrix));
   check(n);
   for(let i=0;i<3*n;i++)maxFloatError=Math.max(maxFloatError,Math.abs(floats[i]-general[i]/65536));
  }
  assert.ok(maxFloatError<=2/65536,'Exhibit float/fixed error exceeds two Q16 units');
  const stats=Object.fromEntries(paths.map(p=>[p,summary(samples[p])]));
  result.runs.push({scene,detail,vertices:n,oneTimePreparationMs:oneTime,
   planPreparationMs:summary(planSamples),floatMatrixPreparationMs:summary(floatMatrixSamples),
   poseEligibility,maxFloatError,summaryMs:stats,
   speedupVsFloat:Object.fromEntries(fixedPaths.map(p=>[p,stats.before.median/stats[p].median])),
   speedupVsCurrent:Object.fromEntries(fixedPaths.filter(p=>p!=='after').map(p=>[p,stats.after.median/stats[p].median])),
   samples});
 }
 result.correctness.checkedComponents=checkedComponents;
 result.correctness.checksum=checksum.toString(16).padStart(8,'0');
 console.log(JSON.stringify(result,null,2));
})().catch(error=>{console.error(error);process.exitCode=1;});
