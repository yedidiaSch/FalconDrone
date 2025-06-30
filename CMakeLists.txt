cmake_minimum_required(VERSION 3.15)

project(FalconDrone LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE INTERNAL "")

# Create the FreeRTOS library manually
add_library(FreeRTOS-Kernel
    include/FreeRTOS/FreeRTOS/Source/event_groups.c
    include/FreeRTOS/FreeRTOS/Source/list.c
    include/FreeRTOS/FreeRTOS/Source/queue.c
    include/FreeRTOS/FreeRTOS/Source/stream_buffer.c
    include/FreeRTOS/FreeRTOS/Source/tasks.c
    include/FreeRTOS/FreeRTOS/Source/timers.c
    include/FreeRTOS/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    include/FreeRTOS/FreeRTOS/Source/portable/MemMang/heap_4.c
)

target_include_directories(FreeRTOS-Kernel PUBLIC
    include/FreeRTOS/FreeRTOS/Source/include
    include/FreeRTOS/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${FREERTOS_CONFIG_FILE_DIRECTORY}
)

add_library(FreeRTOS::Kernel ALIAS FreeRTOS-Kernel)

add_executable(${PROJECT_NAME}
    src/main.cpp
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${FREERTOS_CONFIG_FILE_DIRECTORY}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        FreeRTOS::Kernel
)
