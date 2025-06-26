cmake_minimum_required(VERSION 3.15)

project(FalconDrone LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# FreeRTOS kernel via FetchContent
include(FetchContent)
FetchContent_Declare(
  FreeRTOS
  GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
  GIT_TAG        V10.5.1
)
FetchContent_MakeAvailable(FreeRTOS)

add_executable(${PROJECT_NAME}
    src/main.cpp
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        include
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        FreeRTOS::Kernel
)
