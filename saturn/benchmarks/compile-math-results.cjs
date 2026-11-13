/* Publish only validated native SH-2 math observations. MIT. */
const fs=require('node:fs'),path=require('node:path');
const {validate}=require('./math-model/validate.cjs');
function compile(input,output){
  const data=validate(JSON.parse(fs.readFileSync(input,'utf8')));
  fs.writeFileSync(output,
    '// Generated from validated SH-2 instruction-model evidence; no browser timings.\n'+
    'window.SH2ZAM_MATH_MODEL='+JSON.stringify(data).replace(/</g,'\\u003c')+';\n');
  console.log('Published native SH-2 math comparisons:',data.records.length);
  return data;
}
if(require.main===module){
  const root=path.resolve(__dirname,'../..');
  compile(process.argv[2]||path.join(__dirname,'math-model/results.json'),
    process.argv[3]||path.join(root,'exhibits/math-results.js'));
}
module.exports={compile};
