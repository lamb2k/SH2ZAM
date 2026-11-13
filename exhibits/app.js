/* SH2ZAM interactive exhibit. MIT, Copyright (c) 2026 SH2ZAM Contributors. */
'use strict';
(() => {
  const $ = id => document.getElementById(id), TAU = Math.PI * 2;
  const reduced = matchMedia('(prefers-reduced-motion: reduce)');
  const canvases = [$('before-canvas'), $('after-canvas')];
  const contexts = canvases.map(c => c.getContext('2d'));
  let wasm, mesh, input, before, after, width = 1000, height = 490;
  let scene = 'saturn', detail = 3, seconds = 0, playing = !reduced.matches;
  let busy = false, last = 0, report = null, maxError = 0;
  const specs = {
    saturn: ['Ring world', 'A low-poly planet and concentric rings, transformed in lockstep.'],
    cubes: ['Cube field', 'A lattice of independent cubes sharing one batched transform.'],
    torus: ['Twisted orbit', 'A continuous torus knot: more vertices, the same camera and math.']
  };
  function makeMesh(kind, level) {
    const vertices = [], faces = [];
    const vertex = (x,y,z) => { vertices.push(x,y,z); return vertices.length/3-1; };
    const tri = (a,b,c,color=0) => faces.push({indices:[a,b,c],color});
    const quad = (a,b,c,d,color=0) => { tri(a,b,c,color); tri(a,c,d,color); };
    if(kind === 'saturn') {
      const rows=8+level*3, cols=16+level*8;
      for(let j=0;j<=rows;j++) for(let i=0;i<cols;i++) {
        const p=j*Math.PI/rows, t=i*TAU/cols;
        vertex(1.38*Math.sin(p)*Math.cos(t),1.38*Math.cos(p),1.38*Math.sin(p)*Math.sin(t));
      }
      for(let j=0;j<rows;j++) for(let i=0;i<cols;i++) {
        const a=j*cols+i,b=j*cols+(i+1)%cols;
        quad(a,b,b+cols,a+cols,j%4===0?1:0);
      }
      const bands=[[1.82,2.05],[2.12,2.48],[2.55,2.7]];
      for(let band=0;band<bands.length;band++) {
        const offset=vertices.length/3, steps=cols*2;
        for(let i=0;i<steps;i++) for(const r of bands[band]) {
          const t=i*TAU/steps; vertex(r*Math.cos(t),r*Math.sin(t)*0.36,r*Math.sin(t)*0.933);
        }
        for(let i=0;i<steps;i++) {
          const a=offset+i*2,b=offset+((i+1)%steps)*2;
          quad(a,b,b+1,a+1,band===1?2:1);
        }
      }
    } else if(kind === 'cubes') {
      const n=level+2, step=4.1/n, r=step*0.29;
      for(let x=0;x<n;x++) for(let y=0;y<n;y++) for(let z=0;z<n;z++) {
        const o=vertices.length/3,c=[(x-(n-1)/2)*step,(y-(n-1)/2)*step,(z-(n-1)/2)*step];
        for(const v of [[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]]) vertex(c[0]+r*v[0],c[1]+r*v[1],c[2]+r*v[2]);
        for(const f of [[0,3,2,1],[4,5,6,7],[0,1,5,4],[2,3,7,6],[0,4,7,3],[1,2,6,5]]) quad(...f.map(i=>o+i),(x+y+z)%3);
      }
    } else {
      const rows=48+level*24, cols=6+level*2;
      const center=t=>[(1.65+0.52*Math.cos(3*t))*Math.cos(2*t),(1.65+0.52*Math.cos(3*t))*Math.sin(2*t),0.52*Math.sin(3*t)];
      for(let i=0;i<rows;i++) {
        const t=i*TAU/rows,c=center(t),a=center(t+0.001),v=a.map((x,j)=>x-c[j]);
        const length=Math.hypot(...v); for(let j=0;j<3;j++) v[j]/=length;
        let normal=[-v[1],v[0],0], nl=Math.hypot(...normal);
        if(nl<0.001) { normal=[1,0,0]; nl=1; }
        normal=normal.map(x=>x/nl);
        const b=[v[1]*normal[2]-v[2]*normal[1],v[2]*normal[0]-v[0]*normal[2],v[0]*normal[1]-v[1]*normal[0]];
        for(let j=0;j<cols;j++) { const p=j*TAU/cols; vertex(...c.map((x,k)=>x+0.25*(Math.cos(p)*normal[k]+Math.sin(p)*b[k]))); }
      }
      for(let i=0;i<rows;i++) for(let j=0;j<cols;j++) quad(i*cols+j,((i+1)%rows)*cols+j,((i+1)%rows)*cols+(j+1)%cols,i*cols+(j+1)%cols,Math.floor(i/8)%3);
    }
    return {vertices,faces};
  }
  function invalidate() {
    report=null; $('before-time').textContent=$('after-time').textContent='—';
    $('export').disabled=true; $('result').textContent='Ready to measure. Browser timings only; no Saturn performance claim.';
  }
  function loadScene() {
    mesh=makeMesh(scene,detail);
    const count=mesh.vertices.length/3;
    if(count>wasm.capacity()) throw new Error('Mesh exceeds the WebAssembly buffer.');
    input=new Int32Array(wasm.memory.buffer,wasm.input_ptr(),count*3);
    input.set(mesh.vertices.map(v=>Math.round(v*65536)));
    if(!wasm.prepare(count)) throw new Error('The C bridge rejected the mesh.');
    before=new Float32Array(wasm.memory.buffer,wasm.before_ptr(),count*3);
    after=new Int32Array(wasm.memory.buffer,wasm.after_ptr(),count*3);
    $('scene-title').textContent=specs[scene][0]; $('scene-caption').textContent=specs[scene][1];
    $('mesh-count').textContent=`${count.toLocaleString()} vertices / ${mesh.faces.length.toLocaleString()} triangles`;
    invalidate(); draw();
  }
  function resize() {
    const box=$('stage').getBoundingClientRect(); width=box.width; height=box.height;
    const dpr=Math.min(devicePixelRatio||1,2);
    canvases.forEach((canvas,i)=>{canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);contexts[i].setTransform(dpr,0,0,dpr,0,0);});
    if(wasm) draw();
  }
  function project(x,y,z) { const s=height*1.25/(8-z); return [width/2+x*s,height/2-y*s]; }
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
  function draw() {
    if(!wasm||!mesh) return;
    const phase=seconds/12, ax=Math.round(5200+3000*Math.sin(phase*TAU))&65535,ay=Math.floor(phase*65536)&65535;
    const x=ax*TAU/65536,y=ay*TAU/65536;
    wasm.setup(ax,ay,Math.sin(x),Math.cos(x),Math.sin(y),Math.cos(y));
    wasm.before(); wasm.after();
    const a=render(contexts[0],before,1),b=render(contexts[1],after,65536);
    maxError=0; for(let i=0;i<a.length;i++) maxError=Math.max(maxError,Math.hypot(a[i][3]-b[i][3],a[i][4]-b[i][4]));
    $('error').textContent=`${maxError.toFixed(3)} px`;
    $('time').value=seconds; $('time-value').textContent=`${seconds.toFixed(2)} s`;
    $('play').textContent=playing?'Pause':'Play';$('play-state').textContent=playing?'12 SECOND LOOP':'PAUSED / SCRUB TO INSPECT';
    window.SH2ZAM_EXHIBIT_STATE={ready:true,scene,detail,count:input.length/3,maxError,seconds,playing,busy};
  }
  function frame(now) {
    if(last&&playing&&!busy&&!document.hidden) { seconds=(seconds+Math.min((now-last)/1000,0.1))%12;draw(); }
    last=now; requestAnimationFrame(frame);
  }
  function wipe(value) {
    const v=Math.max(0,Math.min(100,Number(value)));
    $('wipe').value=v;$('wipe-value').textContent=`${Math.round(v)}%`;
    $('before-canvas').style.clipPath=`inset(0 ${100-v}% 0 0)`;$('divider').style.left=`${v}%`;
  }
  const median=xs=>[...xs].sort((a,b)=>a-b)[Math.floor(xs.length/2)];
  const yieldFrame=()=>new Promise(resolve=>requestAnimationFrame(resolve));
  async function benchmark() {
    if(busy||!wasm) return; busy=true; playing=false; draw();
    const controls=[...document.querySelectorAll('button,input')];controls.forEach(c=>c.disabled=true);
    const a=[],b=[],iterations=64;
    try {
      for(let i=0;i<32;i++){wasm.before();wasm.after();}
      const measure=fn=>{const start=performance.now();for(let i=0;i<iterations;i++)fn();return (performance.now()-start)/iterations;};
      for(let i=0;i<21;i++) {
        $('result').textContent=`Measuring paired transform batches ${i+1}/21…`;await yieldFrame();
        if(i%2){b.push(measure(wasm.after));a.push(measure(wasm.before));}else{a.push(measure(wasm.before));b.push(measure(wasm.after));}
      }
      const ta=median(a),tb=median(b);
      $('before-time').textContent=ta>0?`${ta.toFixed(4)} ms`:'< timer limit';
      $('after-time').textContent=tb>0?`${tb.toFixed(4)} ms`:'< timer limit';
      const ratio=ta>0?tb/ta:null;
      $('result').textContent=ratio===null?'Timer resolution is insufficient; increase mesh detail.':`SH2ZAM takes ${ratio.toFixed(2)}× the float reference time in this browser. Not a Saturn result.`;
      report={schema:1,backend:'SH2ZAM portable C / WebAssembly',reference:'float32 C / WebAssembly',scene,detail,vertices:input.length/3,triangles:mesh.faces.length,loopSeconds:seconds,maximumScreenErrorPx:maxError,iterationsPerSample:iterations,sampleCount:21,medianMs:{before:ta,after:tb},samplesMs:{before:a,after:b},wasmSHA256:window.SH2ZAM_WASM_SHA256,userAgent:navigator.userAgent,measuredAt:new Date().toISOString(),excludes:['geometry generation','matrix setup','rendering','output conversion'],saturnBenchmark:false};
    } catch(error) { $('result').textContent=`Measurement failed: ${error.message}`; }
    finally { busy=false;controls.forEach(c=>c.disabled=false);$('export').disabled=!report;draw(); }
  }
  $('wipe').addEventListener('input',e=>wipe(e.target.value));
  const drag=$('drag');
  drag.addEventListener('pointerdown',e=>{drag.setPointerCapture(e.pointerId);});
  drag.addEventListener('pointermove',e=>{if(drag.hasPointerCapture(e.pointerId)){const r=$('stage').getBoundingClientRect();wipe((e.clientX-r.left)/r.width*100);}});
  $('play').addEventListener('click',()=>{playing=!playing;last=0;draw();});
  $('time').addEventListener('input',e=>{playing=false;seconds=Number(e.target.value);invalidate();draw();});
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
      $('backend').textContent='ACTUAL SH2ZAM C · PORTABLE BACKEND';
      $('play').disabled=$('benchmark').disabled=false;
      loadScene();resize();requestAnimationFrame(frame);
    } catch(error) { $('backend').textContent='EXHIBIT NOT LOADED';$('failure').hidden=false;$('failure').textContent=error.message;$('result').textContent='No fallback simulation is shown.';window.SH2ZAM_EXHIBIT_STATE={ready:false,error:error.message}; }
  }
  wipe(50);init();
})();
