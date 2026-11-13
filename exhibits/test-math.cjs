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
  const a=new Float32Array(w.memory.buffer,w.before_ptr(),n*3),b=new Int32Array(w.memory.buffer,w.after_ptr(),n*3);
  let maximum=0,checks=0;
  for(let frame=0;frame<120;frame++){
    const x=(frame*211)&65535,y=(frame*547)&65535,rx=x*Math.PI*2/65536,ry=y*Math.PI*2/65536;
    w.setup(x,y,Math.sin(rx),Math.cos(rx),Math.sin(ry),Math.cos(ry));w.before();w.after();
    for(let i=0;i<a.length;i++){assert.ok(Number.isFinite(a[i]));const error=Math.abs(a[i]-b[i]/65536);maximum=Math.max(maximum,error);assert.ok(error<0.001,'bounded-scene precision drift');checks++;}
  }
  w.setup(0,0,0,1,0,1);w.before();w.after();
  for(let i=0;i<input.length;i++)assert.equal(b[i],input[i],'fixed-point identity');
  console.log(JSON.stringify({checks,maximumWorldError:maximum,identity:'pass',imports:0}));
})().catch(e=>{console.error(e);process.exit(1);});
