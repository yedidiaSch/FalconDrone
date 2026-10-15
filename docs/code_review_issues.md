# FalconDrone Code Review: Issues to Check

Reviewed 2026-10-14 on branch `feature/attitude-control` (commit `5a7a6bc1`).
Files: `Core/Src/pid.c`, `mixer.c`, `pwm_esc.c`, `freertos.c`, `MPU6050.cpp`, `cpp_adapter.cpp`, `telemetry.py`, `visual.py`.
This was a read-only review. Nothing was built, flashed or run on the board.

A second pass on the same day re-checked every item against the code. Each issue has a verdict:
- **REAL**: confirmed in the code and needs a fix.
- **NEEDS BENCH**: correctness depends on hardware that can't be confirmed from code.
- **OK**: exists, but no fix is needed now.
- **WRONG**: my first-pass claim was incorrect.

## Design summary

- **Tasks:** Default (LED), IMU (100 Hz), Control (sem-synced to IMU), Log (UART drain).
- **Pipeline:** IMU sample -> mailbox queue (`imuWaitSample`) -> 3x rate PID -> X-frame mixer -> `ESC_SetAllMotors`.
- **PID:** P on error, I clamped to `i_limit`, D on measurement, output clamped to `out_limit`.
- **Mixer:** `M1 = T - r - p - y`, `M2 = T + r + p - y`, `M3 = T + r - p + y`, `M4 = T - r + p + y`, each clamped to 1000-2000 us.
- **IMU:** complementary filter (ALPHA 0.98), DLPF ~42 Hz, gyro bias from manual B1 calibration.

## Summary table

| # | Issue | Verdict | Priority |
|---|---|---|---|
| 1a | Roll loop sign | NEEDS BENCH (likely correct) | high |
| 1b | Pitch: mixer header comment is backwards | REAL (comment only) | low |
| 2 | Yaw loop is likely positive feedback | REAL if sensor is Z-up, NEEDS BENCH | **high** |
| 3 | "Self-leveling" comment is wrong (rate-only loop) | **FIXED** (comment); angle loop still TODO | medium |
| 4 | `base_throttle` 1300 from boot, no arm gate | **FIXED**: 1300 only applies while ARMED | high (safety) |
| 5 | No arm/disarm state machine | **FIXED** (B1 arm, `arming.c`); RC failsafe still TODO | medium |
| 6 | Per-motor clamping in mixer | OK for now | low |
| 7 | Integral clamp-only anti-windup | OK for now (Ki=0) | low |
| 8 | `dt` handling | OK, drift claim WRONG | none |
| 9 | Telemetry: `telemetry.py` fine, `visual.py` broken | Partly WRONG / REAL (`telemetry.py` now also survives USB loss) | low |
| 10 | `build/` tracked in git | REAL, but `.gitignore` already has it | low |
| 11 | NEW: no gyro bias until B1 pressed | **FIXED**: every arm calibrates first and rejects motion | medium |
| 12 | NEW: `MPU_BLOCK_SIZE` declared as `float` | REAL (smell) | low |
| 13 | NEW: `MPU6050::_sem` captured before the semaphore exists | REAL (dead field) | low |
| 14 | Tooling: `CMakePresets.json` needs Ninja, which isn't installed | REAL | low |
| 15 | Tooling: `launch.json` points to a missing SVD file | REAL | low |
| 16 | Tooling: `copilot-instructions.md` is out of date | REAL | low |
| 17 | NEW (found on hardware): stuck I2C bus never recovered; IMU dead until the board is unplugged | **FIXED**: recover in `Init()` and on a failed DMA start | high |
| 18 | NEW (found on hardware): logger overwrote the DMA buffer mid-transfer, garbling lines | **FIXED**: wait for the DMA before taking the next message | medium |

## Update 2026-10-14: arm/disarm implemented

- **New module:** `Core/Src/arming.c`. Boot state is DISARMED and the motors stay at 1000 us.
- **To arm:** hold B1 for 2 s. The board then calibrates the gyro (and fails if it moves), and checks for 0.5 s that it is level (within ±15°) and still (under 5 dps).
- **Disarm triggers:** a B1 press, `d` or space typed in `telemetry.py` (sent over USART2 RX), an IMU timeout of 100 ms, or tilt beyond 60°.
- **Telemetry:** the CSV gained an 11th field, `arm` (0 = disarmed, 1 = armed, 2 = arming).
- **LD2 blink rate:** 1 s when disarmed, 250 ms while arming, 100 ms when armed.
- **Testing:** the logic passed 13 host-side scenarios. It is not yet verified on hardware.

## Sign analysis (basis for 1 and 2)

The analysis assumes the MPU6050 is mounted with **X forward, Y left, Z up** (chip face up). With the right-hand rule:
- `+gyro_x` means right side down (roll right).
- `+gyro_y` means **nose down**.
- `+gyro_z` means **counter-clockwise** viewed from above.

The PID output is `-Kp * gyro` (setpoint 0), so the controller asks for the opposite of the measured rotation.

| Axis | Gyro positive means | Mixer +cmd actually does | Loop result |
|---|---|---|---|
| Roll | roll right | raises M2/M3 (left), so rolls right | **negative feedback (correct)** |
| Pitch | nose down | raises M2/M4 (rear), so nose **down** | **negative feedback (correct)** |
| Yaw | CCW | raises M3/M4 (CCW props), whose reaction torque yaws the body **CW** | **positive feedback (wrong)** |

If the sensor is mounted differently (Z down, or rotated), the table changes. That is why every axis still needs a bench check.

## Issues

### 1a. Roll sign: NEEDS BENCH (likely correct)
- With an X-forward mounting the roll loop opposes the motion.
- **Check correction:** this is a *rate* loop. A static tilt produces no response, so the motors react only *while the board is rotating*. Roll the board right and, during the motion, M1/M4 (right side) should go up and M2/M3 down.

### 1b. Pitch header comment is backwards: REAL (comment only)
- `mixer.h` says `pitch_cmd > 0 -> pitch up (nose climbs)`, but `+pitch` raises the rear motors (M2, M4), which pitches the nose **down**.
- The loop itself works out correct (see the table). Only the documentation is wrong.
- **Fix:** change the comment to `pitch_cmd > 0 -> nose down`, or flip the pitch signs in the mixer *and* negate `gyro_y` together.

### 2. Yaw is likely positive feedback: REAL if Z-up, NEEDS BENCH
- The mixer treats `+yaw` as CW, while a Z-up gyro reports CCW as positive. The PID therefore commands more rotation in the direction the body is already turning.
- With props on, this would spin the drone up in yaw.
- **Check:** props off. Rotate the body CCW by hand. The correct response is the **CW pair (M1, M2) going down** and the CCW pair (M3, M4) going up. If M1/M2 go up instead, the loop is inverted.
- **Fix if confirmed:** negate yaw in the mixer, or feed `-gyro_z` into the yaw PID. Pick one place, and comment the convention.

### 3. Rate-only, not self-leveling: REAL
- `freertos.c:251` says "setpoint = 0 -> self-leveling". With gyro-rate feedback it only damps rotation and never returns to level.
- `s.roll` and `s.pitch` are only logged.
- **Fix:** correct the comment now. Add an outer angle loop (angle error -> rate setpoint) before flight.

### 4. `base_throttle = 1300` from boot: REAL (intentional)
- Motors spin at about 30% about 4 s after power-up, with no arm gate. The commit message says this is deliberate for symmetric bench response.
- **Never fit props with this build.**
- **Fix:** gate it behind an arm state (see 5).

### 5. No arm/disarm state machine: REAL
- `ESC_Disarm()` and `ESC_Arm()` are never called. Grep shows no callers outside `pwm_esc.c`.
- The IMU timeout path only drops to `ESC_MIN_US`, which is armed idle, not off.
- There is no RC input, so there is no loss-of-signal handling.

### 6. Per-motor clamping in the mixer: OK for now
- This is confirmed in `mixer.c`. With `out_limit` 300 per axis around a 1300 base, one motor can be asked for up to 1300 ± 900, so it saturates at 1000 or 2000 when two or three axes push hard together. When that happens the axis ratios are distorted.
- It is rare at bench levels. Revisit at hover throttle (shift all four outputs, or scale the axis commands, to preserve the ratios).

### 7. Anti-windup: OK for now
- The integral is clamped to ±200 but not gated on output saturation. Ki is 0 on all axes, so it has no effect today.

### 8. `dt` handling: OK, drift claim WRONG
- Both loops use 1 ms ticks (`osKernelGetTickCount` and `HAL_GetTick`), which are the same SysTick-derived base. My first-pass "the two loops could drift" was wrong.
- `last_tick` is not refreshed on the control timeout path, which is harmless thanks to the clamp.
- Minor: the IMU sets `dt = 0` when `dt > 0.2 s`. This happens on the first sample and after a recalibration, which freezes the integrator for one step. Harmless.

### 9. Telemetry tooling: partly WRONG, partly REAL
- `telemetry.py` expects exactly the 10 columns the firmware sends. That works, so my first-pass concern was wrong.
- "PID observability" in the last commit refers to the symmetric throttle base, not to logging PID terms. That claim was also wrong.
- **REAL:** `visual.py:39` still parses `roll, pitch` (2 fields) and will throw on the current 10-field lines.
- **Fix:** update `visual.py` to take fields 1 and 2, or delete it if it is obsolete.

### 10. `build/` tracked: FIXED (2026-10-15, untracked on `chore/housekeeping`)
- `.gitignore` already lists `build/`, but 49 files under `build/` were committed earlier, so git keeps tracking them.
- **Fix:** `git rm -r --cached build && git commit`. Do not add a new ignore rule.

### 11. NEW: zero gyro bias until B1 is pressed: REAL
- Calibration is manual only (by design, per the commit). Until B1 is pressed the bias is 0, and the typical MPU6050 offset of ±2 dps feeds into the PID as a constant rate error. That gives a steady motor offset (about 1.2 us roll and pitch, about 3 us yaw at current gains). It becomes a real drift once Ki is non-zero.
- **Fix:** log a warning until calibration has run, or block arming until it has run (ties into 5).

### 12. NEW: `MPU_BLOCK_SIZE` is a `float`: FIXED (2026-10-15)
- `MPU6050.h:90` has `static constexpr float MPU_BLOCK_SIZE = 14;`, which is passed to `HAL_I2C_Mem_Read_DMA` as a `uint16_t`. It works through implicit conversion but is the wrong type.
- **Fix:** `static constexpr uint16_t`.

### 13. NEW: `MPU6050::_sem` holds a NULL handle: FIXED (2026-10-15)
- `static MPU6050 imu(&hi2c1, imuSemHandle);` in `cpp_adapter.cpp` runs during C++ static init, before `osSemaphoreNew`, so `_sem` is NULL.
- It is harmless today because `imuTick()` uses the global `imuSemHandle` directly, but it is a trap if anyone uses `_sem` later.
- **Fix:** remove the constructor parameter, or assign it in `imuInit()`.

## Build / flash tooling

Scanned 2026-10-14. The toolchain is complete and `cmake --build build` works:
- `arm-none-eabi-gcc` 13.2, cmake 3.28 + make, `st-flash`/`st-info` 1.8.0, openocd 0.12, gdb-multiarch.
- ST-Link udev rules are installed and the user is in `plugdev`.
- The `venv/` has pyserial, matplotlib and numpy.
- The `stm32-flash` skill covers build → probe → flash → telemetry.

The current image uses 76 KB of flash and 22 KB of RAM. Flashing was not tested because the board was disconnected.

### 14. `CMakePresets.json` uses Ninja, which isn't installed: REAL
- The presets set `"generator": "Ninja"` and `binaryDir: build/<preset>`. Ninja is not installed, so `cmake --preset Debug` fails.
- `build/Debug/` holds only a half-configured cache from an earlier attempt.
- The working build directory is `build/`, configured with Unix Makefiles. The VS Code task, the skill and the copilot doc all assume it.
- **Fix:** either `sudo apt install ninja-build` and move everything to `build/Debug`, or change the preset generator to `"Unix Makefiles"`. Then delete `build/Debug/`.

### 15. `launch.json` points to a missing SVD file: REAL
- `.vscode/launch.json` sets `svdFile: ${workspaceFolder}/stm32f446.svd`, but no SVD file exists in the repo.
- Debugging with OpenOCD and Cortex-Debug still works. Only the peripheral register view is missing.
- **Fix:** download `STM32F446.svd` from ST (or the cmsis-svd repo) and add it to the repo with matching case, or remove the `svdFile` line.

### 16. `.github/copilot-instructions.md` is out of date: REAL
- It does not mention ControlTask, the PID, the mixer or ESC output, and it says TIM2/TIM3 are "reserved for future PWM".
- Its Build & Flash section has only `cd build && cmake .. && make`. There is no flash step, and the `cmake ..` call omits the toolchain file.
- It recommends `visual.py`, which is broken (see 9). It should point to `telemetry.py`.
- **Fix:** update the architecture, pin table, build/flash commands (`st-flash --reset write build/falconDrone.bin 0x08000000`) and the telemetry section.

### Related items already listed
- `visual.py` is broken with the 10-field format (see 9).
- `build/` is tracked despite `.gitignore` (see 10).

### How to run (reference)
```bash
cmake --build build --parallel 4                               # build
st-info --probe                                                # check board
st-flash --reset write build/falconDrone.bin 0x08000000        # flash + reset
./venv/bin/python telemetry.py --port /dev/ttyACM0 \
    --record logs/session_$(date -u +%Y%m%dT%H%M%SZ).csv       # telemetry
```
To rebuild `build/` from scratch: `cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug`.

**After boot:** the ESCs arm for about 4 s, then the motors run at about 1300 µs (**props off**). Press B1 while holding the board still to calibrate the gyro.

## Suggested order

1. **Bench-check yaw (2)** with props off, then roll and pitch (1a). Fix the signs or comments (1b, 3).
2. Add an arm/disarm state that gates `base_throttle`, requires calibration, and uses `ESC_Disarm()` (4, 5, 11).
3. Tune gains: Kp, then Kd, then Ki, per axis.
4. Add the angle loop (3) and RC input.
5. Housekeeping: untrack `build/` (10), fix `visual.py` (9), fix the type and the dead field (12, 13), handle mixer saturation (6).
6. Tooling: fix the presets (14), the SVD file (15) and the copilot doc (16).

## Update 2026-10-15: `cpp_adapter` design review, all fixed

`cpp_adapter.{h,cpp}` was renamed to **`imu_service.{h,cpp}`**. It had grown from a thin C-to-C++ shim into the IMU service: task loop, calibration, DMA sync, recovery and data hand-off. The C/C++ boundary is still needed because CubeMX generates `main.c` and `freertos.c` as C.

| # | Issue | Fix |
|---|---|---|
| 19 | **Bug:** `HAL_I2C_ErrorCallback` released the same semaphore as success, so a failed read was decoded and fed to the PID | The error callback sets `s_dma_error`; the service drops the sample and recovers the bus |
| 20 | `dt` came from `HAL_GetTick()` (1 ms resolution, ±10% at 10 ms) and the loop was `tick + osDelay(10)` (period drifted past 10 ms) | Fixed-rate `osDelayUntil` loop owned by the service. Each sample carries its scheduled slot time `t_ms`, so filter and PID `dt` are exact (10 ms, or 20 ms after a dropped read). Resyncs after the ~1 s calibration instead of bursting. |
| 21 | Double-precision `atan2`/`sqrt` linked (software-emulated on the single-precision M4F FPU) | `std::atan2`/`std::sqrt` with float arguments. The binary now links only `atan2f`/`sqrtf`. |
| 22 | Blocking `printf` in `imuInit` raced LogTask's UART DMA | `Log_Print` |
| 23 | Hidden coupling: the adapter `extern`-declared `hi2c1` and two semaphores owned by `freertos.c` (`imuDataReadySem` was hand-added inside a CubeMX-generated block) | `imuCreate(&hi2c1)` injects the bus. The service creates its own static semaphore and mailbox. `imuSem` was removed from `freertos.c` and the `.ioc`. The I2C HAL callbacks forward to `imuOnI2cRxComplete`/`imuOnI2cError`. |
| 24 | Semaphore + global object + `taskENTER_CRITICAL` snapshot | A single-slot FreeRTOS queue used as a mailbox (`xQueueOverwrite` / `xQueueReceive`): wake-up and a by-value copy in one primitive |
| 25 | C API: `imuInit()` with empty parens (unspecified args in C), redundant `extern "C"` around self-guarded headers | `(void)` signatures; redundant wrappers removed |
| 26 | `MPU6050` class: public mutable fields, fusion inside the driver, `#include "i2c.h"` (board coupling), duplicate recovery branches | Driver now does bytes to units only (`Decode()` returns `ImuSample`; `Bias()` getter). Fusion moved to `ComplementaryFilter` (`attitude_filter.{h,cpp}`), the seam for Madgwick. The driver includes only `stm32f4xx_hal.h`; bus recovery is the service's job. The filter seeds from the accel angle on the first sample. |
| 27 | Static object with a runtime-dependent constructor | `constexpr` default constructor; the handle is given in `Init()`. No `_GLOBAL__sub_I_*` constructors in the ELF. |

DMA timeouts now abort the in-flight DMA stream before `I2C_Recover`.

**Status:** builds clean with no warnings. **Not yet verified on hardware** (the board was disconnected). On the next flash, check:
- `[IMU] Initialization Success!`
- CSV `t_ms` steps of 20 ms (50 Hz logging)
- arming still reaches ARMED
- unplugging SDA gives `[IMU] I2C error` / `DMA timeout` and a disarm, not garbage angles

### 28. `telemetry.py` lost ~7 s of data at startup: FIXED (2026-10-15)
- Serial was read only inside the matplotlib animation callback. While the window was being created, nothing drained the port, so the 4 KB tty buffer overflowed. Seen as a 7.5 s jump in `t_ms` in the recorded CSV.
- **Fix:** a `SerialReader` thread starts right after the port opens. It records CSV lines itself and passes decoded lines to the GUI through a queue. Tested on a pty with a 4 s GUI stall: no gaps.

## Update 2026-10-16: watchdog, reset cause, brown-out

No fault protection existed: a hung task left the motors at their last PWM, and a reboot left no trace of why.

| # | Gap | Fix |
|---|---|---|
| 29 | No watchdog | Supervised IWDG (`watchdog.{h,c}`), ~0.5 s timeout. IMUTask (2 s deadline, covers the 1 s calibration) and ControlTask (300 ms) check in each loop. DefaultTask supervises every 50 ms and refreshes the IWDG only if every task is on time. The stalled task and its silent time are saved in a `.noinit` RAM record that survives the reset. Frozen while a debugger halts the core. |
| 30 | Reset cause unknown | `reset_info.{h,c}`: `RCC->CSR` latched as the first line of `main()`, then cleared. Boot log: `[SYS] Reset cause: ...`, e.g. `WATCHDOG - ControlTask silent 301 ms`. |
| 31 | BOR off (factory default): on a sagging BEC the MCU runs down to ~1.7 V with unreliable flash/I2C | BOR option byte set to level 3 (~2.7 V) on first boot; later boots only verify it. Only the BOR_LEV bits are written. |

Debug builds only: sending `h` over the UART makes ControlTask hang on purpose (ignored unless DISARMED).

**Verified on hardware (2026-10-16):**
- Boot report shows the reset cause (`RESET PIN` after `st-flash reset`).
- 25 s normal run with no spurious reset.
- `h` → `[SYS] Reset cause: WATCHDOG - ControlTask silent 301 ms` after the reboot.
- Option byte at `0x1FFFC000` reads `0xE3` (BOR_LEV = 00 = level 3; RDP still `0xAA`).
