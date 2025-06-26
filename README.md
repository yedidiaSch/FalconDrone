# FalconDrone

Minimal real-time quadcopter control system built with FreeRTOS and C++ on STM32.

This repository provides a small skeleton project using **CMake** and **VS Code**. It targets STM32 Nucleo boards via the `arm-none-eabi` toolchain and pulls in the FreeRTOS kernel automatically with CMake's `FetchContent`.

## Building

```
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake
cmake --build build
```

The first command configures the project using the cross compilation toolchain. The second command builds the firmware.

## VS Code

The `.vscode/` folder contains minimal settings for the CMake extension and a `build` task to invoke the compile step.

## Files

- `src/main.cpp` – example FreeRTOS application creating a single LED task.
- `include/FreeRTOSConfig.h` – basic FreeRTOS configuration for Cortex‑M.
- `cmake/arm-none-eabi-gcc.cmake` – toolchain definition for `arm-none-eabi`.

Use this as a starting point for your own drone firmware.
