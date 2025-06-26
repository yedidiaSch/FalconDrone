set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(TOOLCHAIN_PREFIX arm-none-eabi)

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy CACHE INTERNAL "objcopy tool")
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}-size CACHE INTERNAL "size tool")

set(CPU_OPTIONS "-mcpu=cortex-m4" "-mthumb" "-mfpu=fpv4-sp-d16" "-mfloat-abi=hard")

add_compile_options(
    ${CPU_OPTIONS}
    -O2
    -ffunction-sections
    -fdata-sections
)

add_link_options(
    ${CPU_OPTIONS}
    -Wl,--gc-sections
)
