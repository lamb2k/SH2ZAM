/* Partition correctness demo, not a dual-CPU performance measurement. */
#include "math_examples.h"
#include <stdio.h>
int main(void) {
    if(!sh2z_example_batch()) return 1;
    puts("Two disjoint jobs match the serial transform for all 17 vertices.");
    return 0;
}
