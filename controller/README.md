# FalconDrone Controller (ESP32)

Handheld radio controller for FalconDrone: a slider for throttle, a joystick for roll/pitch with its push button as the arm button, an SSD1306 OLED for status, and an nRF24L01+ radio talking to the drone.

## Layout

```
controller/            ESP-IDF project
├── main/
│   ├── main.c         app_main, task creation
│   └── board.h        pin map + wiring notes
└── sdkconfig.defaults tracked config (sdkconfig itself is generated)
shared/
└── rc_protocol/       RcPacket / RcAck — the same header is used by the STM32 firmware
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

Expected boot output:

```
I (...) controller: FalconDrone controller boot
I (...) controller: RcPacket 10 bytes, RcAck 6 bytes, channel 108, 50 Hz
```

## Wiring

See `main/board.h`. Everything runs on **3.3 V**. Put a 10–100 µF capacitor across the nRF24's VCC/GND pins, right at the module.
