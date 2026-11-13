/* Publish validated native SH-2 workloads; never synthesize measurements. MIT. */
'use strict';
const fs=require('node:fs'),path=require('node:path');
const {validate}=require('./workload-model/validate.cjs');

function compile(input,output,{allowMissing=false}={}) {
  let text;
  try { text=fs.readFileSync(input,'utf8'); }
  catch(error) {
    if(!allowMissing || error.code!=='ENOENT') throw error;
    fs.writeFileSync(output,
      '// No native workload capture is present. No performance figures are supplied.\n'+
      'window.SH2ZAM_WORKLOAD_MODEL=null;\n');
    console.log('Native workload capture pending; published explicit unavailable state.');
    return null;
  }
  // Validate before opening the destination. A rejected capture cannot replace
  // the last usable bundle or become a set of plausible-looking UI numbers.
  const data=validate(JSON.parse(text));
  fs.writeFileSync(output,
    '// Generated from validated native SH-2 instruction-model evidence; setup included.\n'+
    'window.SH2ZAM_WORKLOAD_MODEL='+JSON.stringify(data).replace(/</g,'\\u003c')+';\n');
  console.log('Published native SH-2 workload comparisons:',data.records.length);
  return data;
}

if(require.main===module) {
  try {
    const args=process.argv.slice(2),allowMissing=args.includes('--allow-missing');
    const paths=args.filter(arg=>arg!=='--allow-missing');
    if(paths.length>2 || paths.some(arg=>arg.startsWith('--')))
      throw new Error('Usage: node compile-workload-results.cjs [input.json] [output.js] [--allow-missing]');
    const root=path.resolve(__dirname,'../..');
    compile(paths[0]||path.join(__dirname,'workload-model/results.json'),
      paths[1]||path.join(root,'exhibits/workload-results.js'),{allowMissing});
  } catch(error) {
    console.error('Workload publication rejected: '+error.message);process.exitCode=1;
  }
}
module.exports={compile};
