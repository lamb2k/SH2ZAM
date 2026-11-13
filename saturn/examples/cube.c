/* Host preview; feed these positions to your renderer, not directly to VRAM. */
#include "math_examples.h"
#include <stdio.h>
int main(void) {
    sh2z_screen_t vertices[8]; uint8_t visible;
    if(!sh2z_example_cube(sh2z_angle_from_degrees(35),vertices,&visible)||visible!=255u) return 1;
    for(unsigned i=0;i<8;i++) printf("vertex %u: (%d, %d)\n",i,(int)vertices[i].x,(int)vertices[i].y);
    return 0;
}
