# FalconDrone Controller (ESP32)

Handheld radio controller for FalconDrone: a throttle potentiometer (knob or slider), a joystick for roll/pitch, an ARM switch and an ANGLE/RATE mode switch, and an SSD1306 OLED for status. It talks to the drone over ESP-NOW (default, through the bridge ESP32 on the drone, see `bridge/README.md`) or an nRF24L01+.

## Layout

```
controller/                ESP-IDF project
├── main/
│   ├── main.c             app_main + radio task (50 Hz send, read drone status)
│   ├── inputs.c           slider / stick / button → packet values, calibration constants
│   ├── radio_link.h       radio transport interface
│   ├── radio_espnow.c     ESP-NOW transport (pairs with the bridge automatically)
│   ├── radio_nrf24.c      nRF24 transport
│   ├── nrf24_port_esp32.c nRF24 board glue (SPI3, CE pin)
│   ├── Kconfig.projbuild  menuconfig: FalconDrone → Radio transport
│   ├── ui.c / oled.c      screen task (10 Hz) + SSD1306 over I2C
│   ├── ui_draw.c          screen layout (pure C)
│   ├── gfx.c, font5x7.h   framebuffer, text, bars
│   └── board.h            pin map + wiring notes
├── test/                  host tests for the screen: make -C controller/test
└── sdkconfig.defaults     tracked config (sdkconfig itself is generated)
shared/
├── rc_protocol/           RcPacket / RcAck + rc_frame framing — also used by the STM32 firmware
├── espnow_wifi/           Wi-Fi + ESP-NOW startup shared with the bridge
└── nrf24/                 portable nRF24L01+ driver (+ host tests)
```

## Setup (once)

1. In VS Code, install the **ESP-IDF** extension (publisher: Espressif).
2. Run **ESP-IDF: Configure ESP-IDF Extension** → Express → latest stable release.
3. Serial access without sudo (log out and back in afterwards):
   `sudo usermod -aG dialout $USER`

## Radio transport

Picked in `idf.py menuconfig` → **FalconDrone → Radio transport**:

| Transport | Controller hardware | Drone side | STM32 build |
|---|---|---|---|
| **ESP-NOW** (default) | none extra (built-in Wi-Fi) | bridge ESP32 on USART1, see `bridge/` | `FALCON_RC_RADIO=BRIDGE` (default) |
| nRF24 | nRF24L01+ on VSPI | nRF24 on SPI2 | `FALCON_RC_RADIO=NRF24` |

Both sides must use the same transport. Switching the STM32: `cmake --preset Debug -DFALCON_RC_RADIO=NRF24`.

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
I (...) espnow: ESP-NOW up on Wi-Fi channel 1, my MAC ... — looking for the bridge
I (...) controller: radio ready (ESP-NOW)
I (...) espnow: paired with bridge 24:6f:28:...
I (...) controller: thr=   0 roll=   +0 pitch=   +0 btn=0 | tx 100% | drone DISARMED thr=1000us rx=100%
```

`tx` = packets the drone's radio (nRF24) or the bridge (ESP-NOW) acknowledged, `rx` = packets the flight controller received. Without the drone the line reads `no reply from drone` and shows raw ADC values, which is what you need to calibrate.

## Screen

```
DISARMED            RATE    state (inverted when ARMED), drone's flight mode
THR [█████    ]  45%  ┌──┐  slider
OUT 1000us            │ ·│  throttle the drone is commanding
TX 100% RX  98%       └──┘  packets acked by the drone / received by it; stick box
ARM SWITCH TO ARM           what to do next (inverted = warning)
```

- **Top line:** `NO RADIO` means the controller's own radio didn't start. `NO LINK` means the drone isn't replying.
- **Bottom line without a link:** `SEARCHING FOR DRONE` = nothing on the other end answers (bridge / drone radio off or out of range). `NO REPLY FROM DRONE` = the radio on the drone gets our packets but the STM32 doesn't answer (UART wiring, STM32 not running, wrong transport built).
- **Mode (top right):** the drone's mode. It blinks while the MODE switch asks for the other one; the drone takes it at the next disarm.
- **`ARM SW OFF, THEN ON`:** the ARM switch is ON but the drone didn't arm (it was already ON when the link came up, the throttle was up, or arming failed). Flip it OFF and ON again.
- **Preview without hardware:** `make -C controller/test screens` writes every screen as an image to `controller/test/screens/`.
- **Display problems:**
  - Picture upside down: set `OLED_ROTATE_180` in `main/oled.c`.
  - Picture shifted by 2 px, or garbage: the display is probably an SH1106 (common on 1.3" modules) rather than an SSD1306.
  - Without a display, the controller still works and logs a warning.

## Calibration

Constants at the top of `main/inputs.c`:

- `SLIDER_RAW_MIN` / `SLIDER_RAW_MAX`: raw values at the throttle pot's two ends. Set `SLIDER_INVERT` if "up" reads lower. Each read drops the highest and lowest of 8 ADC samples; the zero threshold sits above the remaining jitter (~250 raw).
- `ARM_SW_ON_LEVEL` / `MODE_ANGLE_LEVEL`: GPIO level of each switch in its ON / ANGLE position. It depends on which end pin went to GND; the log shows `raw ... arm= mode=`.
- `STICK_INVERT_X` / `STICK_INVERT_Y`: set so stick right gives `roll > 0` and stick back (toward you) gives `pitch > 0`.
- The stick center is measured at every boot, so don't touch the stick while powering up.

## Wiring

See `main/board.h` (controller). With ESP-NOW the controller needs no radio wiring at all. The drone side is in `bridge/main/board.h` (ESP-NOW) or `Core/Inc/nrf24_port_stm32.h` (nRF24: SPI2 on CN10, PB1/PB12-15). Everything runs on **3.3 V** logic. With nRF24, put a 10–100 µF capacitor across each module's VCC/GND pins.

## Using it

- **Arm:** throttle at zero, ARM switch OFF→ON. The drone refuses with the throttle up, and a switch that is already ON never arms: flip it OFF and ON.
- **Disarm:** ARM switch OFF, at any moment. This is the emergency stop.
- **Mode:** MODE switch, ANGLE or RATE. Applied while disarmed only.
- **Failsafe:** the drone disarms if packets stop for 250 ms, or if a controller connects while it is armed from B1.
- **Throttle:** 0..1000 maps to 1000..`THROTTLE_MAX_US` µs, slew-limited.
- **Stick button:** sent to the drone (`RC_FLAG_BUTTON`) but has no action yet.
- **Stick:** full stick = `ANGLE_MAX_DEG` tilt in ANGLE mode, `STICK_RATE_MAX_DPS` in RATE mode.

## Dry test (no props)

1. Drone on USB with `telemetry.py`: expect `[RC] Radio ready (bridge)`, then `[RC] Link up` once the controller runs.
2. `$?` prints `[RC] ... thr= roll= pitch=`. Move each input and check the sign.
3. Throttle up, ARM switch ON: `[ARM] Refused: throttle not at zero`.
4. Throttle to zero, ARM switch OFF→ON: ARMED. The throttle moves `thr` and the motors. ARM OFF: `DISARMED: arm switch off`.
5. MODE switch while disarmed: `[MODE] ANGLE (controller switch)`. While armed: nothing until the next disarm.
6. RATE mode, stick right: `rsp` positive and the left-side motors speed up. Check pitch the same way.
7. While armed, unplug the controller: `[RC] Link lost`, then `DISARMED: radio link lost`.
