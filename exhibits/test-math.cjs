/* Run after build.sh: node exhibits/test-math.cjs */
const fs=require('node:fs'),assert=require('node:assert/strict');
(async()=>{
  const bytes=fs.readFileSync(__dirname+'/math.wasm');
  const module=await WebAssembly.compile(bytes);
  assert.equal(WebAssembly.Module.imports(module).length,0,'No hidden host imports');
  const w=(await WebAssembly.instantiate(module,{})).exports;
  assert.equal(w.capacity(),16384);assert.equal(w.prepare(16385),0);assert.equal(w.prepare(0),1);
  const n=2048,input=new Int32Array(w.memory.buffer,w.input_ptr(),3*n);
  let seed=12345;const rand=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;};
  for(let i=0;i<input.length;i++)input[i]=Math.round((rand()*2-1)*3*65536);
  assert.equal(w.prepare(n),1);
  assert.notEqual(w.general_ptr(),w.after_ptr(),'separate output buffers');
  const a=new Float32Array(w.memory.buffer,w.before_ptr(),n*3),g=new Int32Array(w.memory.buffer,w.general_ptr(),n*3),b=new Int32Array(w.memory.buffer,w.after_ptr(),n*3);
  let maximum=0,generalMaximum=0,checks=0;
  for(let frame=0;frame<120;frame++){
    const x=(frame*211)&65535,y=(frame*547)&65535,rx=x*Math.PI*2/65536,ry=y*Math.PI*2/65536;
    w.setup(x,y,Math.sin(rx),Math.cos(rx),Math.sin(ry),Math.cos(ry));w.before();w.general();w.after();
    assert.equal(w.candidate_available(),1,'bounded exhibit selects prepared scalar, not native SH-2 candidates');
    for(let i=0;i<a.length;i++){
      assert.ok(Number.isFinite(a[i]));
      assert.equal(b[i],g[i],'prepared is bit-exact with the prior safe batch');
      const error=Math.abs(a[i]-b[i]/65536),generalError=Math.abs(a[i]-g[i]/65536);
      maximum=Math.max(maximum,error);generalMaximum=Math.max(generalMaximum,generalError);
      assert.ok(error<2/65536,'shared-coefficient transform precision drift');checks++;
    }
  }
  w.setup(0,0,0,1,0,1);w.before();w.general();w.after();
  for(let i=0;i<input.length;i++){
    assert.equal(b[i],input[i],'optimized identity');
    assert.equal(g[i],input[i],'general identity');
    assert.equal(a[i],input[i]/65536,'float identity');
  }
  // The timed inclusive wrapper and kernel-only diagnostic must agree.
  const expected=b.slice();b.fill(0);w.after_kernel();
  assert.deepEqual(b,expected);
  // Fresh preparation of full-range coordinates must select the general
  // fallback, including INT32_MIN's unsigned magnitude, and stay exact.
  for(let i=0;i<input.length;i++)input[i]=i%3===0?-2147483648:i%3===1?2147483647:i*65537;
  assert.equal(w.prepare(n),1);w.setup(8192,9216,0,0,0,0);
  assert.equal(w.candidate_available(),0,'full-range mesh fails the no-clamp proof');
  w.general();w.after();assert.deepEqual(b,g,'prepared fallback preserves full-range safe output');
  const {makeMesh}=require('./meshes.js');
  let fixtureChecks=0,reusedPoses=0;
  for(const scene of ['saturn','cubes','torus'])for(let detail=1;detail<=5;detail++){
    const points=makeMesh(scene,detail).vertices.map(value=>Math.round(value*65536));
    const count=points.length/3;
    const source=new Int32Array(w.memory.buffer,w.input_ptr(),points.length);source.set(points);
    assert.equal(w.prepare(count),1);
    const ref=new Float32Array(w.memory.buffer,w.before_ptr(),points.length);
    const safe=new Int32Array(w.memory.buffer,w.general_ptr(),points.length);
    const optimized=new Int32Array(w.memory.buffer,w.after_ptr(),points.length);
    for(let pose=0;pose<21;pose++){
      const ax=(5200+Math.round(3000*Math.sin(pose/21*Math.PI*2)))&65535;
      const ay=Math.floor(pose/21*65536)&65535;
      w.setup(ax,ay,0,0,0,0);w.before();w.general();w.after();
      assert.equal(w.optimized_path(),scene==='cubes'?2:1,'generic coordinate counts select the intended strategy');
      assert.equal(w.reuse_block_size(),scene==='cubes'?8:0,'actual coordinate pattern selects the eight-vertex loop');
      if(w.optimized_path()===2)reusedPoses++;
      for(let i=0;i<points.length;i++){
        assert.equal(optimized[i],safe[i],'reused/prepared output preserves every component and original order');
        assert.ok(Math.abs(ref[i]-optimized[i]/65536)<2/65536);fixtureChecks++;
      }
    }
  }
  console.log(JSON.stringify({checks,fixtureChecks,reusedPoses,maximumWorldError:{general:generalMaximum,optimized:maximum},fixedMismatches:0,identity:'pass',imports:0}));
})().catch(e=>{console.error(e);process.exit(1);});
