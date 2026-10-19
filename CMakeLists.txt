cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME falconDrone)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

file(GLOB LINKER_SCRIPTS "${CMAKE_SOURCE_DIR}/*.ld")
list(LENGTH LINKER_SCRIPTS NUM_SCRIPTS)
if(NUM_SCRIPTS EQUAL 0)
    message(FATAL_ERROR "No .ld linker script found! Please generate it via CubeMX.")
endif()
list(GET LINKER_SCRIPTS 0 LINKER_SCRIPT)
message(STATUS "Using Linker Script: ${LINKER_SCRIPT}")
add_compile_options(${MCU_FLAGS})
add_link_options(${MCU_FLAGS} -T${LINKER_SCRIPT} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections --specs=nosys.specs)



# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/imu_service.cpp
    Core/Src/attitude_filter.cpp
    Core/Src/MPU6050.cpp
    Core/Src/pwm_esc.c
    Core/Src/mixer.c
    Core/Src/pid.c
    Core/Src/flight_ctrl.c
    Core/Src/flight_input.c
    Core/Src/uart_cmd.c
    Core/Src/arming.c
    Core/Src/watchdog.c
    Core/Src/reset_info.c
    Core/Src/logger.c
    Core/Src/rc_link.c
    Core/Src/rc_service.c
    Core/Src/tune_seq.c
    Core/Src/tuning.c
    shared/rc_protocol/src/rc_frame.c
    # Add user sources here
)

# Radio transport: BRIDGE = ESP32 ESP-NOW bridge on USART1, NRF24 = nRF24 on SPI2.
#   cmake --preset Debug -DFALCON_RC_RADIO=NRF24
set(FALCON_RC_RADIO "BRIDGE" CACHE STRING "Radio transport: BRIDGE or NRF24")
set_property(CACHE FALCON_RC_RADIO PROPERTY STRINGS BRIDGE NRF24)
if(FALCON_RC_RADIO STREQUAL "NRF24")
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Core/Src/rc_radio_nrf24.c
        Core/Src/nrf24_port_stm32.c
        shared/nrf24/src/nrf24.c
    )
elseif(FALCON_RC_RADIO STREQUAL "BRIDGE")
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE Core/Src/rc_radio_bridge.c)
else()
    message(FATAL_ERROR "FALCON_RC_RADIO must be BRIDGE or NRF24, got '${FALCON_RC_RADIO}'")
endif()
message(STATUS "FalconDrone radio transport: ${FALCON_RC_RADIO}")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    shared/rc_protocol/include
    shared/nrf24/include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Add linker flags to enable float support in printf
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)

# Post-build: produce raw .bin (for st-flash) and .hex (for STM32CubeProgrammer / dfu)
# alongside the .elf. Sizes are also printed to give a quick flash/RAM read-out.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary  $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex    $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin and ${CMAKE_PROJECT_NAME}.hex"
)