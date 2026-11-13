#include "math_examples.h"
#include <stdio.h>
int main(void) {
    uint32_t failures=sh2z_saturn_smoke();
    printf("Smoke bitmask: 0x%08x; backend=%s\n",(unsigned)failures,sh2z_backend());
    return failures?1:0;
}
