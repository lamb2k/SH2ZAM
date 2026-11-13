# Use Yaul's bare-metal compiler. A Linux SH4 compiler may reject -m2 entirely.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR sh2)
set(SH2Z_TOOLCHAIN_PREFIX sh2eb-elf CACHE STRING "Bare-metal SH-2 toolchain prefix")
set(CMAKE_C_COMPILER ${SH2Z_TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_AR ${SH2Z_TOOLCHAIN_PREFIX}-ar)
set(CMAKE_RANLIB ${SH2Z_TOOLCHAIN_PREFIX}-ranlib)
set(CMAKE_C_FLAGS_INIT "-m2 -mb -O2 -ffreestanding -DSH2Z_REQUIRE_SH2=1")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
