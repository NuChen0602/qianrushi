set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR loongarch64)

set(LOONGARCH_TOOLCHAIN_DIR
    "/opt/ls_2k0300_env/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"
    CACHE PATH "LoongArch cross toolchain directory")

set(CMAKE_C_COMPILER
    "${LOONGARCH_TOOLCHAIN_DIR}/bin/loongarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER
    "${LOONGARCH_TOOLCHAIN_DIR}/bin/loongarch64-linux-gnu-g++")

set(CMAKE_FIND_ROOT_PATH
    "${LOONGARCH_TOOLCHAIN_DIR}/loongarch64-linux-gnu/sysroot")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
