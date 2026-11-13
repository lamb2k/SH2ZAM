/* Correctness only: host execution is not Saturn performance evidence. MIT. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bridge.h"
#include "fixtures.h"
#define CHECK(e) do { if(!(e)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#e); exit(1); } } while(0)
int main(void) {
    double maximum=0;unsigned coordinates=0;
    for(unsigned c=0;c<CASE_COUNT;c++) {
        CHECK(cases[c].count<=capacity());
        memcpy(input_ptr(),cases[c].vertices,cases[c].count*sizeof(sh2z_vec3_t));
        CHECK(prepare(cases[c].count));
        for(unsigned s=0;s<SAMPLE_COUNT;s++) {
            setup(poses[s].ax,poses[s].ay,poses[s].sx,poses[s].cx,poses[s].sy,poses[s].cy);
            CHECK(plan_prepare());
            const unsigned available=candidate_available();
            CHECK((available&1u)!=0);CHECK((available&~7u)==0);
            before();general();after();
            mac_range(0,cases[c].count);dmuls_range(0,cases[c].count);
            sh2z_vec3_t expected[4096];float expected_float[4096*3];
            memcpy(expected,after_ptr(),cases[c].count*sizeof(sh2z_vec3_t));
            memcpy(expected_float,before_ptr(),cases[c].count*3*sizeof(float));
            CHECK(!memcmp(expected,general_ptr(),cases[c].count*sizeof(sh2z_vec3_t)));
            CHECK(!memcmp(expected,mac_ptr(),cases[c].count*sizeof(sh2z_vec3_t)));
            CHECK(!memcmp(expected,dmuls_ptr(),cases[c].count*sizeof(sh2z_vec3_t)));
            CHECK(plan_prepare());after_kernel();
            CHECK(!memcmp(expected,after_ptr(),cases[c].count*sizeof(sh2z_vec3_t)));
            for(unsigned i=0;i<cases[c].count;i+=32) {
                unsigned n=cases[c].count-i;if(n>32)n=32;
                before_range(i,n);general_range(i,n);after_range(i,n);
                mac_range(i,n);dmuls_range(i,n);
            }
            CHECK(candidate_available()==available);
            CHECK(!memcmp(expected,after_ptr(),cases[c].count*sizeof(sh2z_vec3_t)));
            CHECK(!memcmp(expected,general_ptr(),cases[c].count*sizeof(sh2z_vec3_t)));
            CHECK(!memcmp(expected,mac_ptr(),cases[c].count*sizeof(sh2z_vec3_t)));
            CHECK(!memcmp(expected,dmuls_ptr(),cases[c].count*sizeof(sh2z_vec3_t)));
            CHECK(!memcmp(expected_float,before_ptr(),cases[c].count*3*sizeof(float)));
            CHECK(!memcmp(input_ptr(),cases[c].vertices,cases[c].count*sizeof(sh2z_vec3_t)));
            for(unsigned i=0;i<cases[c].count;i++) {
                sh2z_vec3_t v=after_ptr()[i];const int32_t q[]={v.x,v.y,v.z};
                for(unsigned j=0;j<3;j++) {
                    double error=fabs(before_ptr()[3*i+j]-q[j]/65536.0);
                    CHECK(isfinite(error)&&error<0.001);if(error>maximum)maximum=error;
                    coordinates++;
                }
            }
        }
    }
    printf("PASS: 15 shared meshes, 21 poses, five wrappers, chunk/full and four-way fixed bit parity; %u coordinates; max world error %.9f; host fallbacks do not execute SH-2 assembly\n",coordinates,maximum);
}
