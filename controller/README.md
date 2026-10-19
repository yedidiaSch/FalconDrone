# FalconDrone Controller (ESP32)

Handheld radio controller for FalconDrone: a slider for throttle, a joystick for roll/pitch with its push button as the arm button, an SSD1306 OLED for status, and an nRF24L01+ radio talking to the drone.

## Layout

```
controller/                ESP-IDF project
├── main/
│   ├── main.c             app_main + radio task (50 Hz send, read drone status)
│   ├── inputs.c           slider / stick / button → packet values, calibration constants
│   ├── nrf24_port_esp32.c nRF24 board glue (SPI3, CE pin)
│   └── board.h            pin map + wiring notes
└── sdkconfig.defaults     tracked config (sdkconfig itself is generated)
shared/
├── rc_protocol/           RcPacket / RcAck — the same header is used by the STM32 firmware
└── nrf24/                 portable nRF24L01+ driver (+ host tests)
```

## Setup (once)

1. In VS Code, install the **ESP-IDF** extension (publisher: Espressif).
2. Run **ESP-IDF: Configure ESP-IDF Extension** → Express → latest stable release.
3. Serial access without sudo (log out and back in afterwards):
   `sudo usermod -aG dialout $USER`

## Build & flash

Open the `controller/` folder in VS Code (or a terminal with the IDF environment loaded):

```bash
idf.py set-target esp32      # first time only
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Expected output (twice a second once the drone answers):

```
I (...) inputs: stick center x=1890 y=1925
I (...) controller: radio ready, channel 108
I (...) controller: thr=   0 roll=   +0 pitch=   +0 btn=0 | tx 100% | drone DISARMED thr=1000us rx=100%
```

`tx` = packets the drone acknowledged, `rx` = packets the drone received. Without the drone the line reads `no reply from drone` and shows raw ADC values, which is what you need to calibrate.

## Calibration

Constants at the top of `main/inputs.c`:

- `SLIDER_RAW_MIN` / `SLIDER_RAW_MAX`: raw values at the slider's two ends. Set `SLIDER_INVERT` if "up" reads lower.
- `STICK_INVERT_X` / `STICK_INVERT_Y`: set so stick right gives `roll > 0` and stick back (toward you) gives `pitch > 0`.
- The stick center is measured at every boot, so don't touch the stick while powering up.

## Wiring

See `main/board.h` (controller) and `Core/Inc/nrf24_port_stm32.h` (drone: SPI2 on CN10, PB1/PB12-15). Everything runs on **3.3 V**. Put a 10–100 µF capacitor across each nRF24's VCC/GND pins, right at the module.

## Using it

- **Arm:** slider at zero, hold the stick button 2 s. The drone refuses with the slider up.
- **Disarm:** press the stick button.
- **Failsafe:** the drone disarms if packets stop for 250 ms, or if a controller connects while it is armed from B1.
- **Throttle:** slider 0..1000 maps to 1000..`THROTTLE_MAX_US` µs, slew-limited.
- **Stick:** full stick = `ANGLE_MAX_DEG` tilt in ANGLE mode, `STICK_RATE_MAX_DPS` in RATE mode.

## Dry test (no props)

1. Drone on USB with `telemetry.py`: expect `[RC] Radio ready`, then `[RC] Link up` once the controller runs.
2. `$?` prints `[RC] ... thr= roll= pitch=`. Move each input and check the sign.
3. Slider up, hold button: `[ARM] Refused: throttle not at zero`.
4. Slider to zero, hold button 2 s: ARMED. Slider moves `thr` and the motors.
5. RATE mode, stick right: `rsp` positive and the left-side motors speed up. Check pitch the same way.
6. While armed, unplug the controller: `[RC] Link lost`, then `DISARMED: radio link lost`.
