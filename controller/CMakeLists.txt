# FalconDrone handheld controller — ESP32 (ESP-IDF).
cmake_minimum_required(VERSION 3.16)

# Components shared with the STM32 firmware (packet format, later the nRF24 driver).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../shared")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(falcon_controller)
