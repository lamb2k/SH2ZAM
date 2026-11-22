#!/bin/sh
# Mutation controls for test_vehicle_locate.c: each mutated copy of the search (or each extreme stop
# configuration) must make the test fail. Usage: mutate_vehicle_locate.sh [CC]   (run from the saturn directory)
set -u
CC=${1:-gcc}
T=$(mktemp -d)
SRC=src/sh2zam_vehicle_dynamics.c
build() { # name VD_SRC extra...
    name=$1; src=$2; shift 2
    $CC -std=c11 -O2 -Iinclude tests/test_vehicle_locate.c $SRC src/sh2zam_unit30.c src/sh2zam_divu.c \
        -DVD_SRC="\"$src\"" "$@" -lm -o "$T/$name" 2>"$T/$name.err" || { echo "mutant $name: build failed"; cat "$T/$name.err" | head -5; return 1; }
    if "$T/$name" >"$T/$name.out" 2>&1; then
        if [ "$name" = clean ]; then echo "passes   $name"; else echo "SURVIVED $name"; fail=1; fi
    else
        if [ "$name" = clean ]; then echo "FAILED   $name"; fail=1; else echo "caught   $name: $(tail -1 "$T/$name.out")"; fi
    fi
}
mut() { # name sed-expression
    sed "$2" $SRC > "$T/$1.c"; cmp -s $SRC "$T/$1.c" && { echo "mutant $1 did not change the source"; fail=1; return; }
    build "$1" "$T/$1.c"
}
fail=0
# the unmutated source must pass
build clean "$(pwd)/$SRC" || fail=1
build ctl_eager "$(pwd)/$SRC" -DB_FAR=3 -DB_RUN=1
build ctl_far_only "$(pwd)/$SRC" -DB_FAR=2 -DB_RUN=8
mut run_off_by_one 's/if(far_run >= SH2Z_VD_LOCATE_FAR_RUN) {/if(far_run > SH2Z_VD_LOCATE_FAR_RUN) {/'
mut run_not_reset 's/far_run = sumsq(d) > VD_FAR_SQ ? far_run + 1 : 0;/far_run = sumsq(d) > VD_FAR_SQ ? far_run + 1 : far_run;/'
mut square_shift 's/<< 32)$/<< 31)/'
mut stop_returns_minus_one 's/            break;\n//; /SH2Z_VD_LOCATE_FAR_HIT();/{n;s/break;/return -1;/}'
mut run_counts_from_second 's/int far_run = 0;/int far_run = -1;/'
mut no_hit 's/            SH2Z_VD_LOCATE_FAR_HIT();//'
mut far_uses_tolerance 's/far_run = sumsq(d) > VD_FAR_SQ/far_run = sumsq(d) > tolerance/'
rm -rf "$T"
exit $fail
