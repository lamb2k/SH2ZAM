/* Evidence/UI contract checks without collecting browser timings. MIT. */
'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),os=require('node:os'),vm=require('node:vm');
const {compile}=require('../saturn/benchmarks/compile-workload-results.cjs');
const source=fs.readFileSync(path.join(__dirname,'app.js'),'utf8');

class Element {
  constructor(tag='div') {
    this.tagName=tag;this.children=[];this.text='';this.handlers={};this.dataset={};this.style={};this.disabled=false;this.value='';
    const classes=new Set();
    this.classList={add:name=>classes.add(name),remove:name=>classes.delete(name),contains:name=>classes.has(name),
      toggle:(name,on)=>{if(on===undefined)on=!classes.has(name);if(on)classes.add(name);else classes.delete(name);return on;}};
  }
  set textContent(value) {this.text=String(value);this.children=[];}
  get textContent() {return this.text+this.children.map(child=>child.textContent).join('');}
  append(...children) {this.children.push(...children);}
  replaceChildren(...children) {this.text='';this.children=children;}
  addEventListener(name,callback) {this.handlers[name]=callback;}
  dispatch(name,value=this.value) {this.value=value;this.handlers[name]?.({target:this});}
  setAttribute(name,value) {this[name]=value;}
  getAttribute(name) {return this[name];}
  getContext() {return null;}
  closest() {return this.parent||(this.parent=new Element('article'));}
  focus() {this.focused=true;}
  click() {this.dispatch('click');}
}

function render(data) {
  const elements=new Map(),downloads=[];
  const get=id=>{if(!elements.has(id))elements.set(id,new Element());return elements.get(id);};
  const examples=['rotation','spatial'].map(group=>{const element=new Element('button');element.dataset.workloadExample=group;return element;});
  class TestURL extends URL {}
  TestURL.createObjectURL=blob=>{downloads.push(blob);return 'blob:test';};TestURL.revokeObjectURL=()=>{};
  const window={SH2ZAM_WORKLOAD_MODEL:data};
  const document={getElementById:get,createElement:tag=>new Element(tag),
    querySelector:selector=>get(selector),querySelectorAll:selector=>selector==='[data-workload-example]'?examples:[],hidden:false};
  const context={window,document,URL:TestURL,Blob,console,matchMedia:()=>({matches:false,addEventListener(){}}),
    ResizeObserver:class {observe(){}},setTimeout(){},requestAnimationFrame(){},devicePixelRatio:1,WebAssembly};
  vm.runInNewContext(source,context,{filename:'exhibits/app.js'});
  return {window,get,examples,downloads};
}

(async()=>{
  const missing=render(null);
  assert.equal(missing.window.SH2ZAM_WORKLOAD_STATE.ready,false);
  assert.match(missing.get('workload-proof').textContent,/Awaiting validated/);
  assert.equal(missing.get('workload-before-cycles').textContent,'');
  const rejected=render({schema:1});
  assert.equal(rejected.window.SH2ZAM_WORKLOAD_STATE.rejected,true);
  assert.match(rejected.get('workload-status').textContent,/rejected/);

  const temporary=fs.mkdtempSync(path.join(os.tmpdir(),'sh2zam-workload-ui-'));
  try {
    const absent=path.join(temporary,'absent.json'),bundle=path.join(temporary,'bundle.js');
    assert.throws(()=>compile(absent,bundle));
    compile(absent,bundle,{allowMissing:true});
    assert.match(fs.readFileSync(bundle,'utf8'),/SH2ZAM_WORKLOAD_MODEL=null/);
    const input=process.argv[2]||path.resolve(__dirname,'../saturn/benchmarks/workload-model/results.json');
    if(!fs.existsSync(input)) {
      console.log('Workload UI missing/rejected states and missing-capture publication passed; native capture pending.');
      return;
    }
    const data=compile(input,bundle),ui=render(data),state=()=>ui.window.SH2ZAM_WORKLOAD_STATE;
    assert.equal(state().ready,true);
    assert.equal(state().group,'rotation');assert.equal(state().profile,'bounded');assert.equal(state().size,512);
    const row=state().record;
    const best=['scalar','composed','composed_mac'].reduce((a,b)=>row.medianCycles[a]<=row.medianCycles[b]?a:b);
    assert.equal(state().baseline,best,'Headline uses the strongest recorded control');
    assert.equal(state().speedup,row.medianCycles[best]/row.medianCycles.mac);
    assert.equal(ui.get('workload-paths').children.length,5,'Every rotation control remains visible');
    assert.equal(ui.get('workload-samples').children.length,7,'All raw samples remain visible');
    assert.equal(ui.get('workload-all-cases').children.length,44,'Slower and small cases are never filtered out');
    ui.get('workload-baseline').dispatch('change','scalar');
    assert.equal(state().baseline,'scalar');assert.equal(state().speedup,row.medianCycles.scalar/row.medianCycles.mac);
    ui.get('workload-kernel').dispatch('change','batch');
    assert.equal(state().afterCycles,row.medianCycles.batch);
    ui.examples[1].click();
    assert.equal(state().group,'spatial');assert.equal(state().profile,'sparse');assert.equal(state().size,512);
    const spatial=state().record;
    const spatialBest=['api_brute','inline_brute','inline_scatter'].reduce((a,b)=>spatial.medianCycles[a]<=spatial.medianCycles[b]?a:b);
    assert.equal(state().baselineChoice,'strongest');
    assert.equal(state().baseline,spatialBest,'Sparse query example defaults to the fastest of all three controls');
    const independentBest=spatial.medianCycles.inline_brute<=spatial.medianCycles.inline_scatter?'packed output':'sparse hit writes';
    assert.ok(ui.get('workload-spatial-control').textContent.includes(independentBest),'Highlight names the fastest independent C strategy');
    assert.equal(ui.get('workload-paths').children.length,4,'Both independent output strategies and the API control remain visible');
    assert.match(ui.get('workload-scope').textContent,/One index build \+ all 128 queries/);
    for(const record of data.records) {
      ui.get('workload-group').dispatch('change',record.group);
      ui.get('workload-profile').dispatch('change',record.profile);
      ui.get('workload-size').dispatch('change',String(record.size));
      assert.equal(state().record.id,record.id);
      const selected=state().baseline;
      assert.equal(state().speedup,record.medianCycles[selected]/record.medianCycles[state().kernel]);
      assert.equal(ui.get('workload-samples').children.length,7);
      if(state().afterCycles>state().beforeCycles) {
        assert.match(ui.get('workload-speedup').textContent,/control cost/);
        assert.match(ui.get('workload-savings-label').textContent,/EXTRA/);
        assert.equal(ui.get('workload-savings').closest().classList.contains('loss'),true);
      }
    }
    ui.get('workload-export').click();
    const exported=JSON.parse(await ui.downloads[0].text());
    assert.deepEqual(exported,data,'Export retains the full dataset, raw samples, precision and provenance');
    for(const mutate of [
      value=>{value.checkedComparisons++;},
      value=>{value.records[0].medianCycles.scalar++;},
      value=>{value.records[0].samples[0].paths.mac.checksum='deadbeef';},
      value=>{value.run.setupIncluded=false;},
      value=>{value.comparison.claimsUniversalTenfoldGain=true;},
      value=>{value.records.find(record=>record.group==='spatial').paths.splice(2,1);},
      value=>{value.records.pop();}
    ]) {
      const bad=structuredClone(data);mutate(bad);
      assert.equal(render(bad).window.SH2ZAM_WORKLOAD_STATE.ready,false,'Rejected evidence never becomes UI figures');
      const invalid=path.join(temporary,'invalid.json');fs.writeFileSync(invalid,JSON.stringify(bad));
      fs.writeFileSync(bundle,'previous verified bundle');
      assert.throws(()=>compile(invalid,bundle));
      assert.equal(fs.readFileSync(bundle,'utf8'),'previous verified bundle','A rejected capture cannot overwrite valid output');
    }
    console.log('Workload UI passed: 44 native cases, strongest controls including both independent spatial strategies, slower-case labels, seven raw samples, full export, and rejected-data publication.');
  } finally {fs.rmSync(temporary,{recursive:true,force:true});}
})().catch(error=>{console.error(error);process.exitCode=1;});
