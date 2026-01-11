# FalconDrone - RTOS Based Flight Controller

## Project Overview
FalconDrone is an ongoing development of a quadcopter flight control system built from scratch using the **STM32F446RE (Nucleo)** platform. The project focuses on implementing high-performance real-time estimation and control algorithms within a **FreeRTOS** environment.

The current milestone focuses on a "Studio-Tethered" stabilization mode, ensuring the drone maintains its attitude (Roll/Pitch) while secured to a test bench.

## System Architecture
The firmware is designed with a modular, object-oriented approach in C++, emphasizing non-blocking operations and efficient resource management.

### Key Features
* **Real-Time Kernel:** Task management and scheduling via FreeRTOS.
* **Asynchronous I/O:** MPU6050 IMU data acquisition using **I2C DMA** to minimize CPU overhead.
* **Thread-Safe Logging:** Custom UART logging mechanism utilizing **DMA** and FreeRTOS Queues for non-blocking telemetry.
* **Attitude Estimation:** Implementation of raw sensor data processing (Accelerometer and Gyroscope) for Roll/Pitch calculation.

## Hardware Stack
* **MCU:** STM32F446RE (ARM Cortex-M4 @ 180MHz)
* **IMU:** MPU6050 (3-Axis Accelerometer & Gyroscope)
* **Frame:** F450 Quadcopter Frame
* **Motors:** A2212 1000KV Brushless Motors
* **ESCs:** 30A Brushless XXD
* **Power:** 3S Lipo 2200mAh 11.1V

## Software Stack
* **Language:** C++ / C
* **OS:** FreeRTOS
* **Framework:** STM32Cube HAL
* **Build System:** CMake
* **Development Env:** VS Code on Ubuntu

## Current Status: Attitude Estimation Phase
The system currently performs the following:
1.  Initializes hardware peripherals (I2C, UART, DMA, GPIO).
2.  Starts the FreeRTOS scheduler.
3.  Continuously streams 14-byte IMU data packets via DMA.
4.  Processes raw data into Euler angles (Roll and Pitch).

### Forthcoming Milestones
* **Sensor Fusion:** Implementation of a Complementary Filter or Kalman Filter to reduce accelerometer noise and gyroscope drift.
* **PID Control:** Development of the attitude control loop for motor speed regulation.
* **PWM Generation:** Utilizing Timer peripherals to drive ESCs.

## Setup and Installation

### Prerequisites
* GNU Arm Embedded Toolchain
* CMake (minimum 3.10)
* STM32CubeMX (for peripheral configuration)
* OpenOCD / ST-Link Utilities

### Building
```bash
mkdir build
cd build
cmake ..
make