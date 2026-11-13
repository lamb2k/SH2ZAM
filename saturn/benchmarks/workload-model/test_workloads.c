/* Host integration checks for the native workload fixtures. MIT.
 * This validates exact results and fixture isolation, not SH-2 performance. */
#include "suite.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static union workload_output before_outputs[WORKLOAD_PATHS];
static struct workload_input before_inputs;
static struct workload_input sample_inputs[WORKLOAD_SAMPLES];
static uint64_t comparisons,reference_words,sentinel_checks,input_checks,executions;

static void fail(const char *reason,const struct workload_case *c,
                 uint32_t sample,uint32_t path,uint32_t word) {
    fprintf(stderr,"FAIL: %s; case=%s sample=%" PRIu32 " path=%" PRIu32 " word=%" PRIu32 "\n",
            reason,c?c->id:"metadata",sample,path,word);
    exit(EXIT_FAILURE);
}
static uint32_t path_count(const struct workload_case *c) {
    return c->group==W_ROTATION?5u:c->group==W_SPATIAL?4u:2u;
}
static void save_state(void) {
    memcpy(&before_inputs,&workload_inputs,sizeof before_inputs);
    memcpy(before_outputs,workload_outputs,sizeof before_outputs);
}
static void unchanged(const struct workload_case *c,uint32_t sample,
                      uint32_t changed_path,uint32_t changed_words) {
    if(memcmp(&before_inputs,&workload_inputs,sizeof before_inputs)!=0)
        fail("input was mutated",c,sample,changed_path,0);
    ++input_checks;
    for(uint32_t p=0;p<WORKLOAD_PATHS;p++) {
        for(uint32_t w=0;w<WORKLOAD_WORDS;w++) {
            if(p==changed_path&&w<changed_words) continue;
            if(workload_outputs[p].words[w]!=before_outputs[p].words[w])
                fail("inactive output or suffix sentinel was written",c,sample,p,w);
            ++sentinel_checks;
        }
    }
}
static void validate_metadata(const struct workload_case *c) {
    uint32_t expected_words;
    if(c->group>W_SPATIAL||!c->id||!c->size||c->size>WORKLOAD_VECTORS)
        fail("invalid fixture metadata",c,0,0,0);
    if(c->group==W_SPATIAL) {
        if(c->queries!=WORKLOAD_QUERIES||c->size>WORKLOAD_BOXES)
            fail("invalid spatial fixture dimensions",c,0,0,0);
        expected_words=c->queries*((c->size+31u)/32u+1u);
    } else {
        if(c->queries!=0) fail("nonspatial query count",c,0,0,0);
        expected_words=c->size*((c->group==W_ROTATION||c->group==W_NORMALIZE)?3u:1u);
    }
    if(c->words!=expected_words||c->words>WORKLOAD_WORDS)
        fail("wrong fixture output width",c,0,0,0);
}
static void check_prepared_outputs(const struct workload_case *c,uint32_t sample) {
    for(uint32_t p=0;p<WORKLOAD_PATHS;p++) for(uint32_t w=0;w<WORKLOAD_WORDS;w++) {
        if(workload_outputs[p].words[w]!=UINT32_C(0x5a5a5a5a))
            fail("fixture did not reset output sentinels",c,sample,p,w);
        ++sentinel_checks;
    }
}
int main(void) {
    if(workload_case_count!=WORKLOAD_CASES||workload_input_bytes!=sizeof workload_inputs||
       workload_max_words!=WORKLOAD_WORDS)
        fail("wrong exported fixture dimensions",NULL,0,0,0);
    for(uint32_t case_id=0;case_id<WORKLOAD_CASES;case_id++) {
        const struct workload_case *c=&workload_cases[case_id];
        validate_metadata(c);
        for(uint32_t earlier=0;earlier<case_id;earlier++)
            if(strcmp(c->id,workload_cases[earlier].id)==0)
                fail("duplicate case ID",c,0,0,0);
        const uint32_t paths=path_count(c);
        for(uint32_t sample=0;sample<WORKLOAD_SAMPLES;sample++) {
            if(workload_prepare(case_id,sample)!=1) fail("fixture preparation failed",c,sample,0,0);
            check_prepared_outputs(c,sample);
            for(uint32_t earlier=0;earlier<sample;earlier++)
                if(memcmp(&workload_inputs,&sample_inputs[earlier],sizeof workload_inputs)==0)
                    fail("sample inputs were repeated",c,sample,0,0);
            memcpy(&sample_inputs[sample],&workload_inputs,sizeof workload_inputs);
            for(uint32_t p=0;p<paths;p++) {
                save_state();
                if(workload_run(p)!=1) fail("workload path failed",c,sample,p,0);
                ++executions;
                unchanged(c,sample,p,c->words);
                if(p==0) reference_words+=c->words;
                else for(uint32_t w=0;w<c->words;w++) {
                    if(workload_outputs[p].words[w]!=workload_outputs[0].words[w])
                        fail("exact output mismatch",c,sample,p,w);
                    ++comparisons;
                }
            }
            /* A bad dispatch must not change already computed outputs. */
            for(uint32_t p=paths;p<=WORKLOAD_PATHS;p++) {
                save_state();
                if(workload_run(p)!=0) fail("invalid path was accepted",c,sample,p,0);
                unchanged(c,sample,WORKLOAD_PATHS,0);
            }
            save_state();
            if(workload_prepare(WORKLOAD_CASES,sample)!=0||
               workload_prepare(case_id,WORKLOAD_SAMPLES)!=0)
                fail("invalid fixture selection was accepted",c,sample,0,0);
            unchanged(c,sample,WORKLOAD_PATHS,0);
            if(workload_prepare(case_id,sample)!=1||
               memcmp(&workload_inputs,&sample_inputs[sample],sizeof workload_inputs)!=0)
                fail("fixture was not reproducible",c,sample,0,0);
            check_prepared_outputs(c,sample);
        }
    }
    if(executions!=1281||reference_words!=173425||comparisons!=585060)
        fail("incomplete canonical workload coverage",NULL,0,0,0);
    printf("PASS: %u cases x %u samples; %" PRIu64 " path executions; %" PRIu64
           " reference output words; %" PRIu64 " exact path comparisons; %" PRIu64
           " output sentinel checks; %" PRIu64 " input-immutability checks. Host correctness only.\n",
           WORKLOAD_CASES,WORKLOAD_SAMPLES,executions,reference_words,comparisons,sentinel_checks,input_checks);
    return EXIT_SUCCESS;
}
