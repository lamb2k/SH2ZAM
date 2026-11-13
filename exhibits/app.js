/* SH2ZAM interactive exhibit. MIT, Copyright (c) 2026 SH2ZAM Contributors. */
'use strict';
(() => {
  const $ = id => document.getElementById(id), TAU = Math.PI * 2, MAX_WORLD_ERROR = 0.001;
  const reduced = matchMedia('(prefers-reduced-motion: reduce)');
  const canvases = [$('before-canvas'), $('after-canvas')];
  const contexts = canvases.map(c => c.getContext('2d'));
  let wasm, mesh, input, before, beforeBits, general, after, width = 1000, height = 490;
  let scene = 'saturn', detail = 3, seconds = 0, playing = !reduced.matches;
  let busy = false, last = 0, report = null, maxError = 0, maxWorldError = 0, maxGeneralWorldError = 0, fixedMismatches = 0;
  let outputChecksums = {}, outputsValid = false;
  let liveAt = -Infinity, liveOrder = 0;
  let shownStrategy=-1;
  const paths = ['reference','general','optimized'];
  const liveSamples = {reference:[],general:[],optimized:[]};
  const liveIterations = {reference:64,general:64,optimized:64};
  const specs = {
    saturn: ['Ring world', 'A low-poly planet and concentric rings, transformed in lockstep.'],
    cubes: ['Cube field', 'A lattice of independent cubes sharing one batched transform.'],
    torus: ['Twisted orbit', 'A continuous torus knot: more vertices, the same camera and math.']
  };
  const {makeMesh}=window.SH2ZAM_MESHES;
  const strategies=['general-fallback','prepared-scalar','coordinate-reuse'];
  function showOptimizedStrategy() {
    const strategy=wasm.optimized_path();
    if(strategy===shownStrategy)return;
    shownStrategy=strategy;
    $('after-label').textContent=strategy===2?'REUSED PRODUCTS':strategy===1?'PREPARED BATCH':'GENERAL FALLBACK';
    $('stage-after-label').textContent=`${strategy===2?'Reused':'Prepared'} Q16.16 · Browser/WASM`;
    $('optimized-code-title').textContent=strategy===2?'Shared products · Q16.16':'Prepared batch · Q16.16';
    $('optimized-code').textContent=strategy===2?
      '// Tables rebuilt for each matrix.\n// x_term includes raw translation.\nint64_t sum =\n    x_term[row] + yz_term[row];\nreturn (int32_t)\n    sh2zi_prepared_floor16(sum);':
      '// Mesh bounds cached; matrix qualified.\nint64_t sum =\n    (int64_t)row[0] * x\n  + (int64_t)row[1] * y\n  + (int64_t)row[2] * z;\nreturn (int32_t)(\n    sh2zi_prepared_floor16(sum) + row[3]);';
    $('optimized-source').href=`https://github.com/lamb2k/SH2ZAM/blob/main/saturn/src/${strategy===2?'sh2zam_reuse.c':'sh2zam_prepared.c'}`;
    $('optimized-source').textContent=strategy===2?'Coordinate reuse source ↗':'Prepared batch source ↗';
  }
  function invalidate() {
    paths.forEach(path=>{liveSamples[path].length=0;liveIterations[path]=64;}); liveAt=-Infinity;
    $('live-status').textContent='Sampling…';
    $('stage-before-time').textContent=$('stage-after-time').textContent='—';
    report=null; $('before-time').textContent=$('general-time').textContent=$('after-time').textContent='—';
    $('batch-ratio').textContent=$('float-ratio').textContent='—';
    $('plan-time').textContent=$('kernel-time').textContent='Capture to measure';
    $('export').disabled=true; $('result').textContent='Live · browser timings';
  }
  function loadScene() {
    mesh=makeMesh(scene,detail);
    const count=mesh.vertices.length/3;
    if(count>wasm.capacity()) throw new Error('Mesh exceeds the WebAssembly buffer.');
    input=new Int32Array(wasm.memory.buffer,wasm.input_ptr(),count*3);
    input.set(mesh.vertices.map(v=>Math.round(v*65536)));
    if(!wasm.prepare(count)) throw new Error('The C bridge rejected the mesh.');
    before=new Float32Array(wasm.memory.buffer,wasm.before_ptr(),count*3);
    beforeBits=new Uint32Array(wasm.memory.buffer,wasm.before_ptr(),count*3);
    general=new Int32Array(wasm.memory.buffer,wasm.general_ptr(),count*3);
    after=new Int32Array(wasm.memory.buffer,wasm.after_ptr(),count*3);
    $('scene-title').textContent=specs[scene][0]; $('scene-caption').textContent=specs[scene][1];
    $('mesh-count').textContent=`${count.toLocaleString()} vertices / ${mesh.faces.length.toLocaleString()} triangles`;
    invalidate(); draw(); showSaturnResults(); showModelResults();
  }
  function showModelResults() {
    const data=window.SH2ZAM_MODEL_RESULTS,run=data?.run;
    const row=data?.records.find(record=>record.scene===scene&&record.detail===detail);
    $('model-workload').textContent=`${specs[scene][0]} · detail ${detail} · ${input.length/3} vertices · 21 sampled poses`;
    $('model-status').textContent='No model capture';
    $('model-proof').textContent='No model evidence loaded.';
    $('model-provenance').textContent='';
    for(const name of ['reference','safe','prepared','mac','dmuls']) {
      $('model-'+name).textContent='—';$('model-'+name+'-ratio').textContent='—';
    }
    // Isolated guest-cycle evidence never enters the physical-console dataset
    // and is never converted into milliseconds or shown as a live measurement.
    if(!row||data.schema!==1||data.kernelSet!=='float-safe-prepared-fused-sh2-model-v1'||run?.environment!=='isolated-sh2-interpreter')return;
    $('model-status').textContent='Recorded · guest cycles';
    for(const name of ['reference','safe','prepared','mac','dmuls']) {
      $('model-'+name).textContent=row.medianCycles[name].toLocaleString();
      $('model-'+name+'-ratio').textContent=name==='reference'?'1.00×':`${row.referenceOver[name].toFixed(2)}×`;
    }
    $('model-proof').textContent=`All four fixed paths: bit-exact · Max float error ${row.maximumWorldError.toFixed(7)} world units`;
    $('model-provenance').textContent=`Yabause ${run.interpreterCommit.slice(0,8)} · SH-2 build ${run.sourceCommit.slice(0,8)} · flat RAM`;
  }
  function showSaturnResults() {
    const data=window.SH2ZAM_SATURN_RESULTS,run=data?.run;
    const row=data?.records.find(r=>r.scene===scene&&r.detail===detail);
    $('saturn-workload').textContent=`${specs[scene][0]} · detail ${detail} · ${input.length/3} vertices · 21 sampled poses`;
    for(const id of ['saturn-reference','saturn-general','saturn-optimized','saturn-ratio','saturn-float-ratio'])$(id).textContent='—';
    for(const id of ['saturn-reference-cycles','saturn-general-cycles','saturn-optimized-cycles'])$(id).textContent='No console capture';
    $('saturn-error').textContent='Unmeasured';$('saturn-evidence').hidden=true;
    $('saturn-status').textContent='Not measured';
    $('saturn-provenance').textContent='Recorded hardware results will appear here after a console run.';
    // Emulator captures are retained for correctness but never presented as hardware timing.
    if(!row||run?.environment!=='physical-console'||run?.kernelSet!=='float-safe-prepared-fused-v1'||data.schema!==3)return;
    $('saturn-status').textContent='Recorded · physical console';
    $('saturn-reference').textContent=`${row.referenceMs.toFixed(3)} ms`;
    $('saturn-general').textContent=`${row.generalMs.toFixed(3)} ms`;
    $('saturn-optimized').textContent=`${row.optimizedMs.toFixed(3)} ms`;
    $('saturn-reference-cycles').textContent=`${row.referenceCycles.toLocaleString()} cycles`;
    $('saturn-general-cycles').textContent=`${row.generalCycles.toLocaleString()} cycles`;
    $('saturn-optimized-cycles').textContent=`${row.optimizedCycles.toLocaleString()} cycles`;
    $('saturn-ratio').textContent=`${row.generalOverOptimized.toFixed(2)}×`;
    $('saturn-float-ratio').textContent=`${row.referenceOverOptimized.toFixed(2)}×`;
    $('saturn-error').textContent=`Safe / prepared: bit-exact · ${row.preparedAvailable?'Bounds proved':'Full-range fallback'} · Max float error ≤ ${row.maxErrorLSB} Q16 LSB`;
    $('saturn-provenance').textContent=`${run.console} · ${run.videoMode} · ${run.compiler} · ${run.sourceCommit.slice(0,8)} · ${run.capturedAt}`;
    $('saturn-evidence').href=run.evidenceUrl;$('saturn-evidence').hidden=false;
  }
  function resize() {
    const box=$('stage').getBoundingClientRect(); width=box.width; height=box.height;
    const dpr=Math.min(devicePixelRatio||1,2);
    canvases.forEach((canvas,i)=>{canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);contexts[i].setTransform(dpr,0,0,dpr,0,0);});
    if(wasm) draw();
  }
  function project(x,y,z) { const s=Math.min(height,width)*(scene==='cubes'?0.95:1.25)/(8-z); return [width/2+x*s,height/2-y*s]; }
  function render(ctx,points,divisor) {
    ctx.clearRect(0,0,width,height);
    const gradient=ctx.createRadialGradient(width*.5,height*.46,0,width*.5,height*.46,width*.66);
    gradient.addColorStop(0,'#12313d');gradient.addColorStop(1,'#080d15');ctx.fillStyle=gradient;ctx.fillRect(0,0,width,height);
    for(let i=0;i<90;i++) { const x=((i*7919)%1009)/1009*width,y=((i*3571)%1013)/1013*height;ctx.fillStyle=i%4?'#63869777':'#ace9e6aa';ctx.fillRect(x,y,i%6?1:2,1); }
    const pts=[];
    for(let i=0;i<points.length;i+=3) { const x=points[i]/divisor,y=points[i+1]/divisor,z=points[i+2]/divisor;pts.push([x,y,z,...project(x,y,z)]); }
    const sorted=mesh.faces.map(f=>({f,z:f.indices.reduce((sum,i)=>sum+pts[i][2],0)})).sort((a,b)=>a.z-b.z);
    const palette=[[181,73,49],[192,48,56],[163,60,66]];
    for(const {f} of sorted) {
      const [a,b,c]=f.indices.map(i=>pts[i]);
      const u=[b[0]-a[0],b[1]-a[1],b[2]-a[2]],v=[c[0]-a[0],c[1]-a[1],c[2]-a[2]];
      const n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],length=Math.hypot(...n);
      if(length<1e-10) continue;
      const light=0.32+0.68*Math.abs((n[0]*-.35+n[1]*.55+n[2]*.76)/length);
      const [h,s,l]=palette[f.color];ctx.fillStyle=`hsl(${h} ${s}% ${l*light}%)`;
      ctx.beginPath();ctx.moveTo(a[3],a[4]);ctx.lineTo(b[3],b[4]);ctx.lineTo(c[3],c[4]);ctx.closePath();ctx.fill();
      ctx.strokeStyle='#071b2455';ctx.lineWidth=.45;ctx.stroke();
    }
    return pts;
  }
  function checkPrecision() {
    maxWorldError=0;maxGeneralWorldError=0;fixedMismatches=0;
    let refChecksum=0,generalChecksum=0,optimizedChecksum=0;
    for(let i=0;i<after.length;i++) {
      maxWorldError=Math.max(maxWorldError,Math.abs(before[i]-after[i]/65536));
      maxGeneralWorldError=Math.max(maxGeneralWorldError,Math.abs(before[i]-general[i]/65536));
      if(general[i]!==after[i])fixedMismatches++;
      refChecksum=(Math.imul(refChecksum,33)+beforeBits[i])>>>0;
      generalChecksum=(Math.imul(generalChecksum,33)+(general[i]>>>0))>>>0;
      optimizedChecksum=(Math.imul(optimizedChecksum,33)+(after[i]>>>0))>>>0;
    }
    const hex=value=>value.toString(16).padStart(8,'0');
    outputChecksums={reference:hex(refChecksum),general:hex(generalChecksum),optimized:hex(optimizedChecksum)};
    outputsValid=fixedMismatches===0&&[maxError,maxWorldError,maxGeneralWorldError].every(Number.isFinite)
      &&maxWorldError<=MAX_WORLD_ERROR&&maxGeneralWorldError<=MAX_WORLD_ERROR;
    $('error').textContent=Number.isFinite(maxError)?`${maxError.toFixed(3)} px`:'Invalid';
    $('world-error').textContent=Number.isFinite(maxWorldError)?`${maxWorldError.toFixed(7)}`:'Invalid';
    $('fixed-equality').textContent=fixedMismatches?`${fixedMismatches} differing components`:'Safe / optimized: bit-exact';
    $('fixed-equality').classList.toggle('mismatch',fixedMismatches!==0);
  }
  function draw() {
    if(!wasm||!mesh) return;
    const phase=seconds/12, ax=Math.round(5200+3000*Math.sin(phase*TAU))&65535,ay=Math.floor(phase*65536)&65535;
    const x=ax*TAU/65536,y=ay*TAU/65536;
    wasm.setup(ax,ay,Math.sin(x),Math.cos(x),Math.sin(y),Math.cos(y));
    wasm.before(); wasm.general(); wasm.after();
    showOptimizedStrategy();
    const a=render(contexts[0],before,1),b=render(contexts[1],after,65536);
    maxError=0; for(let i=0;i<a.length;i++) maxError=Math.max(maxError,Math.hypot(a[i][3]-b[i][3],a[i][4]-b[i][4]));
    checkPrecision();
    $('play').textContent=playing?'Pause':'Play';$('play-state').textContent=playing?'12 SECOND LOOP':'PAUSED';
    window.SH2ZAM_EXHIBIT_STATE={ready:true,scene,detail,count:input.length/3,maxError,maxWorldError,maxGeneralWorldError,fixedMismatches,outputChecksums,outputsValid,preparedFastPath:!!(wasm.candidate_available()&1),optimizedStrategy:strategies[wasm.optimized_path()],seconds,playing,busy};
  }
  function frame(now) {
    if(last&&playing&&!busy&&!document.hidden) { seconds=(seconds+Math.min((now-last)/1000,0.1))%12;draw(); }
    if(wasm && !busy && !report && !document.hidden && now-liveAt>=500) { sampleLive(); liveAt=now; }
    last=now; requestAnimationFrame(frame);
  }
  const median=xs=>[...xs].sort((a,b)=>a-b)[Math.floor(xs.length/2)];
  function measureAdaptive(fn,targetMs,initialIterations) {
    let iterations=initialIterations,elapsed;
    do {
      const start=performance.now();
      for(let j=0;j<iterations;j++)fn();
      elapsed=performance.now()-start;
      if(elapsed>=targetMs||iterations>=1048576)break;
      iterations=Math.min(1048576,iterations*Math.max(2,Math.min(8,Math.ceil(targetMs/Math.max(elapsed,0.01)))));
    } while(true);
    return {ms:elapsed/iterations,elapsedMs:elapsed,iterations};
  }
  function measurePaths(order,targetMs,iterations) {
    const functions={reference:wasm.before,general:wasm.general,optimized:wasm.after},sample={};
    // Rotate first/second/third position; all three see the same current pose.
    for(let i=0;i<3;i++) {
      const path=paths[(order+i)%3];
      sample[path]=measureAdaptive(functions[path],targetMs,iterations[path]);
      iterations[path]=sample[path].iterations;
    }
    return sample;
  }
  function showTimings(times) {
    const ids={reference:'before-time',general:'general-time',optimized:'after-time'};
    if(!outputsValid) {
      for(const id of [...Object.values(ids),'stage-before-time','stage-after-time','batch-ratio','float-ratio'])$(id).textContent='—';
      return;
    }
    for(const path of paths)$(ids[path]).textContent=times[path]>0?`${times[path].toFixed(4)} ms`:'< timer limit';
    $('stage-before-time').textContent=$('before-time').textContent;
    $('stage-after-time').textContent=$('after-time').textContent;
    $('batch-ratio').textContent=times.general>0&&times.optimized>0?`${(times.general/times.optimized).toFixed(2)}×`:'—';
    $('float-ratio').textContent=times.reference>0&&times.optimized>0?`${(times.reference/times.optimized).toFixed(2)}×`:'—';
  }
  function sampleLive() {
    if(!outputsValid) {showTimings({});$('live-status').textContent='PRECISION CHECK FAILED';$('result').textContent='Measurement failed: transform outputs did not pass precision checks.';return;}
    // Increase repetitions until each live sample spans at least 0.75 ms.
    const sample=measurePaths(liveOrder++,0.75,liveIterations),times={};
    for(const path of paths) {liveSamples[path].push(sample[path].ms);if(liveSamples[path].length>5)liveSamples[path].shift();times[path]=median(liveSamples[path]);}
    showTimings(times);
    for(const id of ['before-time','general-time','after-time'])$(id).dataset.samples=String(liveOrder);
    $('live-status').textContent=`LIVE · SAMPLE ${liveOrder}`;
  }
  const yieldFrame=()=>new Promise(resolve=>requestAnimationFrame(resolve));
  async function benchmark() {
    if(busy||!wasm) return; busy=true; playing=false; report=null; draw();
    const controls=[...document.querySelectorAll('button,input')];controls.forEach(c=>c.disabled=true);
    const samples={reference:[],general:[],optimized:[]},iterations={reference:64,general:64,optimized:64};
    const sampleIterations={reference:[],general:[],optimized:[]},sampleElapsedMs={reference:[],general:[],optimized:[]};
    const componentSamples={planPreparation:[],kernelOnly:[]},componentIterations={planPreparation:64,kernelOnly:64};
    const targetSampleMs=5;
    try {
      if(!outputsValid)throw new Error('Transform outputs did not pass precision checks.');
      for(let i=0;i<32;i++){wasm.before();wasm.general();wasm.after();}
      for(let i=0;i<21;i++) {
        $('result').textContent=`Measuring three transform paths ${i+1}/21…`;await yieldFrame();
        const sample=measurePaths(i,targetSampleMs,iterations);
        for(const path of paths) {
          if(sample[path].elapsedMs<targetSampleMs)throw new Error('Timer resolution is insufficient for a capture.');
          samples[path].push(sample[path].ms);
          sampleIterations[path].push(sample[path].iterations);
          sampleElapsedMs[path].push(sample[path].elapsedMs);
        }
        const components={planPreparation:wasm.plan_prepare,kernelOnly:wasm.after_kernel};
        for(const path of (i%2?['planPreparation','kernelOnly']:['kernelOnly','planPreparation'])) {
          const component=measureAdaptive(components[path],targetSampleMs,componentIterations[path]);
          if(component.elapsedMs<targetSampleMs)throw new Error('Timer resolution is insufficient for component timing.');
          componentSamples[path].push(component);
          componentIterations[path]=component.iterations;
        }
      }
      checkPrecision();
      if(!outputsValid)throw new Error('Transform outputs did not pass precision checks.');
      const times=Object.fromEntries(paths.map(path=>[path,median(samples[path])]));
      const componentTimes=Object.fromEntries(Object.entries(componentSamples).map(([path,values])=>[path,median(values.map(value=>value.ms))]));
      showTimings(times);
      $('plan-time').textContent=`${componentTimes.planPreparation.toFixed(5)} ms`;
      $('kernel-time').textContent=`${componentTimes.kernelOnly.toFixed(4)} ms`;
      $('live-status').textContent='CAPTURED';
      const measurable=paths.every(path=>times[path]>0);
      $('result').textContent=!measurable?'Timer resolution is insufficient; increase mesh detail.':`Captured · safe / optimized: ${(times.general/times.optimized).toFixed(2)}× · browser timings`;
      report={schema:4,kernelSet:'float-safe-mesh-v1',backend:'SH2ZAM exact integer C / WebAssembly',reference:'float32 C / WebAssembly (browser float)',preparedFastPath:!!(wasm.candidate_available()&1),optimizedStrategy:strategies[wasm.optimized_path()],scene,detail,vertices:input.length/3,triangles:mesh.faces.length,loopSeconds:seconds,maximumScreenErrorPx:maxError,maximumWorldError:{general:maxGeneralWorldError,optimized:maxWorldError},maximumAllowedWorldError:MAX_WORLD_ERROR,fixedMismatches,outputChecksums,minimumSampleDurationMs:targetSampleMs,sampleIterations,sampleElapsedMs,sampleCount:21,measurementOrder:'rotate reference/general/optimized',medianMs:times,samplesMs:samples,components:{medianMs:componentTimes,samples:componentSamples},ratios:measurable?{generalOverOptimized:times.general/times.optimized,referenceOverOptimized:times.reference/times.optimized}:null,wasmSHA256:window.SH2ZAM_WASM_SHA256,userAgent:navigator.userAgent,measuredAt:new Date().toISOString(),includes:['matrix qualification and product-table rebuilding per optimized batch','range validation','output writes','JS-to-WASM call'],excludes:['geometry generation','matrix construction and quantization','rendering','input conversion','one-time mesh bounds scan and coordinate indexing','precision checks'],saturnBenchmark:false};
    } catch(error) {if(!outputsValid)showTimings({});$('live-status').textContent='CAPTURE FAILED';$('result').textContent=`Measurement failed: ${error.message}`; }
    finally { busy=false;controls.forEach(c=>c.disabled=false);$('export').disabled=!report;draw(); }
  }
  $('play').addEventListener('click',()=>{playing=!playing;if(playing&&report)invalidate();last=0;draw();});
  $('detail').addEventListener('input',e=>{detail=Number(e.target.value);$('detail-value').textContent=detail;if(wasm)loadScene();});
  document.querySelectorAll('[data-scene]').forEach(button=>button.addEventListener('click',()=>{
    scene=button.dataset.scene;document.querySelectorAll('[data-scene]').forEach(b=>{const active=b===button;b.classList.toggle('active',active);b.setAttribute('aria-pressed',String(active));});if(wasm)loadScene();
  }));
  $('benchmark').addEventListener('click',benchmark);
  $('export').addEventListener('click',()=>{if(!report)return;const url=URL.createObjectURL(new Blob([JSON.stringify(report,null,2)],{type:'application/json'})),a=document.createElement('a');a.href=url;a.download='sh2zam-browser-measurement.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
  reduced.addEventListener('change',e=>{if(e.matches){playing=false;draw();}});
  new ResizeObserver(resize).observe($('stage'));
  async function init() {
    try {
      if(!window.SH2ZAM_WASM_BASE64) throw new Error('Build the exhibit first with bash exhibits/build.sh, or use the ready-to-run CI artifact.');
      const bytes=Uint8Array.from(atob(window.SH2ZAM_WASM_BASE64),c=>c.charCodeAt(0));
      const module=await WebAssembly.instantiate(bytes,{});wasm=module.instance.exports;
      $('backend').textContent='ACTUAL SH2ZAM C · PREPARED SCALAR BACKEND';
      $('play').disabled=$('benchmark').disabled=false;
      loadScene();resize();requestAnimationFrame(frame);
    } catch(error) { $('backend').textContent='EXHIBIT NOT LOADED';$('failure').hidden=false;$('failure').textContent=error.message;$('result').textContent='No fallback simulation is shown.';window.SH2ZAM_EXHIBIT_STATE={ready:false,error:error.message}; }
  }
  init();
})();

