/* Host preview of integer-only sprite positions. */
#include "math_examples.h"
#include <stdio.h>
int main(void) {
    for(uint32_t frame=0;frame<256;frame+=64) {
        sh2z_screen_t p; sh2z_example_orbit(frame,&p);
        printf("frame %u: sprite (%d, %d)\n",(unsigned)frame,(int)p.x,(int)p.y);
        if(p.x<80||p.x>240||p.y<56||p.y>168) return 1;
    }
    return 0;
}
