# SH2ZAM calibration image (Yaul SDK). Build from this directory in ijacquez/yaul.
ifeq ($(strip $(YAUL_INSTALL_ROOT)),)
$(error YAUL_INSTALL_ROOT must point to the Yaul installation)
endif
include $(YAUL_INSTALL_ROOT)/share/build.pre.mk
SH_PROGRAM := sh2zam-calibration
SH_SRCS := main.c ../../../src/sh2zam.c ../../../src/sh2zam_view.c ../../../src/sh2zam_divu.c
BENCH_COMMIT ?= $(shell git rev-parse HEAD 2>/dev/null)
ifeq ($(strip $(BENCH_COMMIT)),)
$(error Set BENCH_COMMIT to the full source commit when git is unavailable)
endif
SH_CFLAGS += -O2 -std=c11 -I../../../include -DSH2Z_REQUIRE_SH2=1 \
  -fno-fast-math -ffp-contract=off -fno-lto -DBENCH_COMMIT=\"$(BENCH_COMMIT)\"
IP_VERSION := V0.100
IP_RELEASE_DATE := 20261117
IP_AREAS := JTUBKAEL
IP_PERIPHERALS := J
IP_TITLE := SH2ZAM calibration
IP_MASTER_STACK_ADDR := 0x06004000
IP_SLAVE_STACK_ADDR := 0x06001E00
IP_1ST_READ_ADDR := 0x06004000
IP_1ST_READ_SIZE := 0
include $(YAUL_INSTALL_ROOT)/share/build.post.iso-cue.mk
