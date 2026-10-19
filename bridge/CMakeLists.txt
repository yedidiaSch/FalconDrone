# FalconDrone radio bridge — ESP32 on the drone: ESP-NOW ⇄ UART ⇄ STM32.
cmake_minimum_required(VERSION 3.16)

# Shared with the controller (packet format, framing, Wi-Fi setup).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../shared")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(falcon_bridge)
