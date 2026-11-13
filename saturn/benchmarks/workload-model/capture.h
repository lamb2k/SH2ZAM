/* SPDX-License-Identifier: GPL-2.0-or-later
 * Execute complete workloads, including shared preparation and output writes.
 * Included only by the optional isolated instruction-model harness. */
#define WORKLOAD_CAPTURE_CASES 44u
#define WORKLOAD_CAPTURE_SAMPLES 7u
#define WORKLOAD_CAPTURE_WORDS 2176u
static const char *workload_groups[]={"rotation","normalize","atan2","atan","spatial"};
static const char *workload_numeric_profiles[]={"bounded","tiny","full-range","identity","zero"};
static const char *workload_spatial_profiles[]={"sparse","mixed","dense","crowded","full-range"};
static const char *workload_rotation_paths[]={"scalar","composed","composed_mac","batch","mac"};
static const char *workload_scalar_paths[]={"prior","current"};
static const char *workload_spatial_paths[]={"api_brute","inline_brute","index"};
static u32 workload_check_output(u32 address,u32 words,const u32 *reference) {
 u32 checksum=UINT32_C(2166136261);
 for(u32 i=0;i<WORKLOAD_CAPTURE_WORDS;i++) {
  u32 value=rl(&cpu,address+4*i);
  if(i<words) {
   if(reference && value!=reference[i]) {
    fprintf(stderr,"Workload word %u: reference=%08x candidate=%08x\n",i,reference[i],value);
    fail("Workload exact output mismatch",address+4*i);
   }
   checksum=(checksum^value)*UINT32_C(16777619);
  } else if(value!=UINT32_C(0x5a5a5a5a)) fail("Workload output outside result",address+4*i);
 }
 return checksum;
}
static void workload_print_cost(struct cost c,u32 checksum) {
 printf("{\"cycles\":%u,\"instructions\":%"PRIu64",\"dataReads\":%"PRIu64",\"dataWrites\":%"PRIu64",\"macInstructions\":%"PRIu64",\"dmulsInstructions\":%"PRIu64",\"checksum\":\"%08x\"}",
  c.cycles,c.instructions,c.reads,c.writes,c.mac_ops,c.dmuls_ops,checksum);
}
static int capture_workloads(void) {
 u32 cases=sym("_workload_cases"),count=rl(&cpu,sym("_workload_case_count"));
 u32 inputs=sym("_workload_inputs"),bytes=rl(&cpu,sym("_workload_input_bytes"));
 u32 outputs=sym("_workload_outputs"),max_words=rl(&cpu,sym("_workload_max_words"));
 u32 reference[WORKLOAD_CAPTURE_WORDS];uint64_t checked=0,comparisons=0;
 if(count!=WORKLOAD_CAPTURE_CASES||bytes>65536||max_words!=WORKLOAD_CAPTURE_WORDS)
  fail("Unexpected workload suite layout",count);
 printf("{\"schema\":1,\"kind\":\"SH2ZAM exact workload optimizations\",\"sourceCommit\":\"%s\",\"compiler\":\"%s\",\"compileFlags\":\"%s\",\"records\":[",
  (char*)ram+offset(sym("_workload_source_commit"),1),
  (char*)ram+offset(sym("_workload_compiler"),1),
  (char*)ram+offset(sym("_workload_compile_flags"),1));
 for(u32 ci=0;ci<count;ci++) {
  u32 base=cases+24*ci,name=rl(&cpu,base),group=rl(&cpu,base+4),profile=rl(&cpu,base+8);
  u32 size=rl(&cpu,base+12),queries=rl(&cpu,base+16),words=rl(&cpu,base+20);
  if(group>4||profile>(group==0||group==4?4u:2u)||!size||size>512||!words||words>max_words)
   fail("Unexpected workload case metadata",ci);
  const char **paths=group==0?workload_rotation_paths:group==4?workload_spatial_paths:workload_scalar_paths;
  u32 path_count=group==0?5u:group==4?3u:2u;
  if(ci) putchar(',');
  printf("{\"id\":\"%s\",\"group\":\"%s\",\"profile\":\"%s\",\"size\":%u,\"queries\":%u,\"outputWords\":%u,\"paths\":[",
   (char*)ram+offset(name,1),workload_groups[group],
   group==4?workload_spatial_profiles[profile]:workload_numeric_profiles[profile],size,queries,words);
  for(u32 p=0;p<path_count;p++) {if(p)putchar(',');printf("\"%s\"",paths[p]);}
  printf("],\"samples\":[");
  for(u32 sample=0;sample<WORKLOAD_CAPTURE_SAMPLES;sample++) {
   if(invoke("_workload_prepare",ci,sample,0,0).result!=1)fail("Workload preparation",ci);
   u32 input_hash=math_hash_bytes(inputs,bytes);
   if(sample)putchar(',');printf("{\"inputChecksum\":\"%08x\",\"paths\":{",input_hash);
   for(u32 p=0;p<path_count;p++) {
    struct cost cost=invoke("_workload_run",p,0,0,0);
    if(cost.result!=1)fail("Workload execution rejected valid fixture",ci);
    if(math_hash_bytes(inputs,bytes)!=input_hash)fail("Workload changed input",ci);
    u32 address=outputs+4*max_words*p;
    u32 checksum=workload_check_output(address,words,p?reference:NULL);
    if(!p)for(u32 w=0;w<words;w++)reference[w]=rl(&cpu,address+4*w);
    if(p)putchar(',');printf("\"%s\":",paths[p]);workload_print_cost(cost,checksum);
   }
   checked+=words;comparisons+=(uint64_t)words*(path_count-1);
   printf("},\"mismatches\":0}");
  }
  /* Compare every word again with SR.S set. Refill each destination so a
   * successful no-op cannot pass, and keep guard calls out of measured costs. */
  u32 input_hash=math_hash_bytes(inputs,bytes);
  for(u32 p=0;p<path_count;p++) {
   u32 address=outputs+4*max_words*p;
   memset(ram+offset(address,4*max_words),0x5a,4*max_words);
   struct cost cost=invoke_sr("_workload_run",p,0,0,0,2);
   if(cost.result!=1||(cost.sr&2)!=2||cost.mac_ops)fail("Workload SR.S fallback",ci);
   (void)workload_check_output(address,words,reference);
   if(math_hash_bytes(inputs,bytes)!=input_hash)fail("Workload guard changed input",ci);
  }
  printf("],\"guards\":{\"srSRetained\":true,\"srSOutputMatch\":true,\"noMacWithS\":true}}");fflush(stdout);
 }
 printf("],\"checkedOutputWords\":%"PRIu64",\"checkedComparisons\":%"PRIu64",\"mismatches\":0}\n",checked,comparisons);
 return 0;
}
