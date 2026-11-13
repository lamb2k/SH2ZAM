# Yaul SDK make integration. Build from this directory in ijacquez/yaul.
ifeq ($(strip $(YAUL_INSTALL_ROOT)),)
$(error YAUL_INSTALL_ROOT must point to the Yaul installation)
endif
include $(YAUL_INSTALL_ROOT)/share/build.pre.mk
SH_PROGRAM := sh2zam-benchmark
SH_SRCS := main.c ../../../exhibits/bridge.c \
  ../../src/sh2zam.c ../../src/sh2zam_trig.c \
  ../../src/sh2zam_angles.c ../../src/sh2zam_transform.c ../../src/sh2zam_geometry.c
BENCH_COMMIT ?= $(shell git rev-parse HEAD)
SH_CFLAGS += -O2 -std=c11 -I../../include -I.. -DSH2Z_REQUIRE_SH2=1 \
  -DSH2Z_EXHIBIT_CAPACITY=4096 -fno-fast-math -ffp-contract=off -fno-lto \
  -DBENCH_COMMIT=\"$(BENCH_COMMIT)\"
IP_VERSION := V0.200
IP_RELEASE_DATE := 20261113
IP_AREAS := JTUBKAEL
IP_PERIPHERALS := J
IP_TITLE := SH2ZAM float vs fixed benchmark
IP_MASTER_STACK_ADDR := 0x06004000
IP_SLAVE_STACK_ADDR := 0x06001E00
IP_1ST_READ_ADDR := 0x06004000
IP_1ST_READ_SIZE := 0
include $(YAUL_INSTALL_ROOT)/share/build.post.iso-cue.mk

