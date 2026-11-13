# Yaul SDK make integration. Build from this directory in ijacquez/yaul.
ifeq ($(strip $(YAUL_INSTALL_ROOT)),)
$(error YAUL_INSTALL_ROOT must point to the Yaul installation)
endif
include $(YAUL_INSTALL_ROOT)/share/build.pre.mk
SH_PROGRAM := sh2zam-benchmark
SH_SRCS := main.c ../../../exhibits/bridge.c \
  ../../src/sh2zam.c ../../src/sh2zam_trig.c \
  ../../src/sh2zam_angles.c ../../src/sh2zam_transform.c ../../src/sh2zam_geometry.c \
  ../../src/sh2zam_prepared.c ../../src/sh2zam_batch_sh2.c ../../src/sh2zam_reuse.c
SH_SRCS += ../math-model/math_bridge.c ../math-model/legacy_sh2zam.c \
  ../math-model/legacy_sh2zam_trig.c ../math-model/legacy_sh2zam_angles.c \
  ../math-model/legacy_sh2zam_transform.c ../math-model/legacy_sh2zam_geometry.c
BENCH_COMMIT ?= $(shell git rev-parse HEAD 2>/dev/null)
ifeq ($(strip $(BENCH_COMMIT)),)
$(error Set BENCH_COMMIT to the full source commit when git is unavailable)
endif
SH_CFLAGS += -O2 -std=c11 -I../../include -I.. -DSH2Z_REQUIRE_SH2=1 \
  -DSH2Z_EXHIBIT_CAPACITY=4096 -fno-fast-math -ffp-contract=off -fno-lto \
  -DBENCH_COMMIT=\"$(BENCH_COMMIT)\"
# The CPU-model harness invokes these entry points directly from the ELF.
# Yaul enables --gc-sections; retain them as linker roots even though the
# console's timing/reporting program does not call the independent baseline.
SH_LDFLAGS += -Wl,--undefined=_plain_prepare,--undefined=_plain_available \
  -Wl,--undefined=_plain_range
SH_LDFLAGS += -Wl,--undefined=_math_prepare,--undefined=_math_before_range \
  -Wl,--undefined=_math_after_range,--undefined=_math_cases \
  -Wl,--undefined=_math_case_count,--undefined=_math_input_bytes \
  -Wl,--undefined=_math_source_commit,--undefined=_math_compiler \
  -Wl,--undefined=_math_compile_flags
IP_VERSION := V0.200
IP_RELEASE_DATE := 20261113
IP_AREAS := JTUBKAEL
IP_PERIPHERALS := J
IP_TITLE := SH2ZAM prepared transform benchmark
IP_MASTER_STACK_ADDR := 0x06004000
IP_SLAVE_STACK_ADDR := 0x06001E00
IP_1ST_READ_ADDR := 0x06004000
IP_1ST_READ_SIZE := 0
include $(YAUL_INSTALL_ROOT)/share/build.post.iso-cue.mk

