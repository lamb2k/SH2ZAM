#!/bin/sh
# Mutation controls for test_vehicle_locate7.c: each mutated copy of the search (or each extreme configuration) must make the test fail. Usage: mutate_vehicle_locate7.sh [CC]   (run from the saturn directory)
set -u
CC=${1:-gcc}
T=$(mktemp -d)
SRC=src/sh2zam_vehicle_dynamics.c
build() { # name VD_SRC extra...
    name=$1; src=$2; shift 2
    $CC -std=c11 -O2 -Iinclude tests/test_vehicle_locate7.c $SRC src/sh2zam_unit30.c src/sh2zam_divu.c \
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
build ctl_damp_one "$(pwd)/$SRC" -DB_DAMP=1
build ctl_flip_one "$(pwd)/$SRC" -DB_FLIP=1
build ctl_flip_two "$(pwd)/$SRC" -DB_FLIP=2
mut step_twice_as_long 's/<= (dt < 0 ? -(int64_t)dt : (int64_t)dt)) {/<= 2 * (dt < 0 ? -(int64_t)dt : (int64_t)dt)) {/'
mut secant_sign 's/vd_div(-(int64_t)along \* prev_dt, da)/vd_div((int64_t)along * prev_dt, da)/'
mut damp_from_second_pass 's/if(++pass > SH2Z_VD_LOCATE_DAMP \&\& prev_dt != 0) {/if(++pass > 0 \&\& prev_dt != 0) {/'
mut secant_slope_sign 's/(da < 0) != (prev_dt < 0)/(da < 0) == (prev_dt < 0)/'
mut no_damp_hook 's/SH2Z_VD_LOCATE_DAMP_HIT(dt, s);//'
mut flip_without_parity 's/ \&\& (iterations \& 1) == 0) {/) {/'
mut flip_no_return_test 's/ \&\& b->t - t2 < VD_FLIP_T \&\& t2 - b->t < VD_FLIP_T//'
mut flip_returns_minus_one 's/SH2Z_VD_LOCATE_FLIP_HIT();/SH2Z_VD_LOCATE_FLIP_HIT(); return -1;/'
mut no_flip_hook 's/SH2Z_VD_LOCATE_FLIP_HIT();//'
rm -rf "$T"
exit $fail
