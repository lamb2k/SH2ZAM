/* SPDX-License-Identifier: GPL-2.0-or-later
 * Optional native math capture, included by sh2-model/harness.c. */
#define MATH_CAPTURE_BATCH 32u
#define MATH_CAPTURE_SAMPLES 7u
#define MATH_CAPTURE_PROFILES 3u
#define MATH_CAPTURE_WORDS 17u
static uint32_t math_hash_bytes(u32 addr,u32 count) {
 uint32_t h=UINT32_C(2166136261);
 for(u32 i=0;i<count;i++) h=(h^rb(&cpu,addr+i))*UINT32_C(16777619);
 return h;
}
static uint32_t math_check_outputs(u32 before,u32 after,u32 words,uint64_t *checked) {
 uint32_t h=UINT32_C(2166136261);
 for(unsigned i=0;i<MATH_CAPTURE_BATCH;i++) for(unsigned j=0;j<MATH_CAPTURE_WORDS;j++) {
  u32 a=rl(&cpu,before+4*(i*MATH_CAPTURE_WORDS+j));
  u32 b=rl(&cpu,after+4*(i*MATH_CAPTURE_WORDS+j));
  if(a!=b) {
   fprintf(stderr,"Math output row %u word %u: previous=%08x optimized=%08x\n",i,j,a,b);
   fail("Math before/after output mismatch",i*MATH_CAPTURE_WORDS+j);
  }
  if(j<words) {h=(h^a)*UINT32_C(16777619);(*checked)++;}
  else if(a!=UINT32_C(0x5a5a5a5a)) fail("Math output outside result",j);
 }
 return h;
}
static int capture_math(void) {
 const char *profiles[MATH_CAPTURE_PROFILES]={"bounded","tiny","full-range"};
 u32 cases=sym("_math_cases"),count=rl(&cpu,sym("_math_case_count"));
 u32 inputs=sym("_math_inputs"),bytes=rl(&cpu,sym("_math_input_bytes"));
 u32 before=sym("_math_before_output"),after=sym("_math_after_output");
 if(count!=48||bytes>65536) fail("Unexpected math suite layout",count);
 uint64_t checked=0;
 printf("{\"schema\":1,\"kind\":\"SH2ZAM previous-versus-optimized math, Yabause isolated SH2 flat-RAM model\",\"batchSize\":%u,\"samplesPerProfile\":%u,",MATH_CAPTURE_BATCH,MATH_CAPTURE_SAMPLES);
 printf("\"sourceCommit\":\"%s\",\"compiler\":\"%s\",\"compileFlags\":\"%s\",\"records\":[",
  (char*)ram+offset(sym("_math_source_commit"),1),
  (char*)ram+offset(sym("_math_compiler"),1),
  (char*)ram+offset(sym("_math_compile_flags"),1));
 for(u32 op=0;op<count;op++) for(u32 profile=0;profile<MATH_CAPTURE_PROFILES;profile++) {
  u32 name=rl(&cpu,cases+8*op),words=rl(&cpu,cases+8*op+4);
  if(!words||words>MATH_CAPTURE_WORDS) fail("Unexpected math result width",words);
  if(op||profile) putchar(',');
  printf("{\"operation\":\"%s\",\"profile\":\"%s\",\"outputWords\":%u,\"samples\":[",(char*)ram+offset(name,1),profiles[profile],words);
  uint32_t final_checksum=0;
  for(u32 sample=0;sample<MATH_CAPTURE_SAMPLES;sample++) {
   if(invoke("_math_prepare",op,profile,sample,0).result!=1) fail("Math fixture preparation",op);
   uint32_t input_hash=math_hash_bytes(inputs,bytes);
   struct cost a=invoke("_math_before_range",0,MATH_CAPTURE_BATCH,0,0);
   struct cost b=invoke("_math_after_range",0,MATH_CAPTURE_BATCH,0,0);
   if(a.result!=1||b.result!=1) fail("Math range failed",op);
   if(math_hash_bytes(inputs,bytes)!=input_hash) fail("Math modified its input",op);
   final_checksum=math_check_outputs(before,after,words,&checked);
   if(sample) putchar(',');
   printf("{\"inputChecksum\":\"%08x\",\"before\":",input_hash);print_cost(a);
   printf(",\"after\":");print_cost(b);
   printf(",\"outputChecksum\":\"%08x\",\"mismatches\":0}",final_checksum);
  }
  /* Exercise native entry/exit with SH-2 saturation mode set, without mixing
   * these guard calls into the measured distributions above. */
  struct cost a=invoke_sr("_math_before_range",0,MATH_CAPTURE_BATCH,0,0,2);
  struct cost b=invoke_sr("_math_after_range",0,MATH_CAPTURE_BATCH,0,0,2);
  uint64_t guard_words=0;
  if(a.result!=1||b.result!=1||math_check_outputs(before,after,words,&guard_words)!=final_checksum)
   fail("Math saturation-mode output mismatch",op);
  printf("],\"srSRetained\":true,\"srSOutputMatch\":true}");fflush(stdout);
 }
 printf("],\"checkedOutputWords\":%"PRIu64",\"mismatches\":0}\n",checked);
 return 0;
}
