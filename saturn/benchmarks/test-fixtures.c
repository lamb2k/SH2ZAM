/* Correctness only: host execution is not Saturn performance evidence. MIT. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "bridge.h"
#include "fixtures.h"
int main(void) {
    double maximum=0;
    for(unsigned c=0;c<CASE_COUNT;c++) {
        assert(cases[c].count<=capacity());
        memcpy(input_ptr(),cases[c].vertices,cases[c].count*sizeof(sh2z_vec3_t));
        assert(prepare(cases[c].count));
        for(unsigned s=0;s<SAMPLE_COUNT;s++) {
            setup(poses[s].ax,poses[s].ay,poses[s].sx,poses[s].cx,poses[s].sy,poses[s].cy);
            before();after();
            sh2z_vec3_t expected[4096];
            memcpy(expected,after_ptr(),cases[c].count*sizeof(sh2z_vec3_t));
            for(unsigned i=0;i<cases[c].count;i+=32) {
                unsigned n=cases[c].count-i;if(n>32)n=32;
                before_range(i,n);after_range(i,n);
            }
            assert(!memcmp(expected,after_ptr(),cases[c].count*sizeof(sh2z_vec3_t)));
            for(unsigned i=0;i<cases[c].count;i++) {
                sh2z_vec3_t v=after_ptr()[i];const int32_t q[]={v.x,v.y,v.z};
                for(unsigned j=0;j<3;j++) {
                    double error=fabs(before_ptr()[3*i+j]-q[j]/65536.0);
                    assert(isfinite(error)&&error<0.001);if(error>maximum)maximum=error;
                }
            }
        }
    }
    printf("PASS: 15 shared meshes, 21 poses, chunk/full parity; max world error %.9f\n",maximum);
}
