---
name: stm32-flash
description: Build, flash, and start telemetry monitoring for the FalconDrone STM32F446 firmware. Trigger when the user wants to deploy firmware to the connected Nucleo board ("flash it", "burn the firmware", "deploy to the board", "push to MCU", "run on the board"). Performs cmake --build → st-info probe → st-flash write at 0x08000000 → background telemetry.py with CSV recording into logs/.
---

# stm32-flash

End-to-end "deploy firmware and start watching it" workflow for FalconDrone.
Always runs in this exact order; abort on any step's failure rather than
retrying or sleeping.

## Steps

### 1. Build

Run `cmake --build build` from the repo root.

- If the build fails, surface the compiler/linker errors and **stop**.
  Do not attempt to flash a stale binary.
- After success, confirm both `build/falconDrone.bin` and
  `build/falconDrone.elf` exist. The CMake post-build step generates `.bin`
  alongside `.elf`; if `.bin` is missing the user is on an old `CMakeLists.txt`
  — point them at it and stop.

### 2. Probe for the ST-Link

Run `st-info --probe`.

- "Found 0 stlink programmers" → the Nucleo isn't connected (or udev rules
  aren't active). Tell the user to plug it in. **Do not poll or sleep** —
  this is a one-shot check. The user will re-invoke when ready.
- Anything else → continue. Note the probed flash size in the report.

### 3. Flash

Run:

```bash
st-flash --reset write build/falconDrone.bin 0x08000000
```

- `0x08000000` is the STM32F4 flash base — never change it for this board.
- Never add `sudo`. The `99-stlink.rules` udev rule and the user's `plugdev`
  group cover permissions.
- Never use `--no-verify` or similar bypass flags.
- On verify failure, report the error and **stop**.

### 4. Wait briefly for the serial port

The Nucleo re-enumerates USB after flashing. Poll `/dev/ttyACM0` for up to
~3 s (e.g. `for i in $(seq 1 6); do [ -e /dev/ttyACM0 ] && break; sleep 0.5; done`).
If it never appears, tell the user and stop — do not start telemetry against
a missing port.

### 5. Launch telemetry in the background, recording CSV

Ensure `logs/` exists (`mkdir -p logs`).

Generate a UTC timestamp filename: `logs/session_$(date -u +%Y%m%dT%H%M%SZ).csv`.

Launch the dashboard via Bash with `run_in_background: true`:

```bash
./venv/bin/python telemetry.py \
    --port /dev/ttyACM0 \
    --record logs/session_<timestamp>.csv
```

- Always use the project venv at `./venv/bin/python` — the system Python
  doesn't have `matplotlib`/`pyserial` installed.
- The window opens via matplotlib; if the user is on a headless session the
  window won't appear but the CSV recording still works. Note this to them
  if they mention SSH/headless.

### 6. Final report

Single concise summary covering:

- Build status (success / RAM and Flash usage from the linker output)
- Probe summary (chip ID / flash size if available)
- Bytes flashed
- CSV record path
- One sentence on next action ("dashboard window should be visible" or
  "tail logs/session_*.csv to inspect the stream")

## Hard rules

- **One-shot, no retries.** If a step fails, stop and report. Don't loop,
  don't `sleep` past 3 s, don't fall back to alternate tools.
- **Never modify firmware as part of this skill.** If a recompile is needed
  the user invokes a normal edit workflow first.
- **Never push or commit** the recorded CSV — `logs/` must stay gitignored.

## Project assumptions baked in

- Build directory: `./build` (CMake)
- Binary: `./build/falconDrone.bin`
- Board: STM32F446RE Nucleo with on-board ST-Link v2.1
- Flash base: `0x08000000`
- Serial: `/dev/ttyACM0` @ 115200 baud (UART2 over the ST-Link VCP)
- Python venv: `./venv/`
- Telemetry script: `./telemetry.py`
- Log directory: `./logs/` (auto-created)
