# Example cross configuration for a libyaul-compatible big-endian SH-2 toolchain.
# Requires sh2eb-elf-gcc on PATH. Compile library only; no console test claim.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR sh2)
set(CMAKE_C_COMPILER sh2eb-elf-gcc)
set(CMAKE_C_FLAGS_INIT "-m2 -mb -O2 -ffunction-sections -fdata-sections")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
