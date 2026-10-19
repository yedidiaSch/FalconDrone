# FalconDrone Radio Bridge (ESP32 on the drone)

Passes the controller's packets from ESP-NOW to the STM32 over UART, and the STM32's replies back. It forwards frames only and never makes up control data, so if it stops, the STM32's 250 ms link timeout disarms the drone.

```
controller ESP32 ──ESP-NOW──► bridge ESP32 ──UART 115200──► STM32 USART1
                 ◄──ESP-NOW──              ◄──UART────────
```

## Wiring

| Bridge (ESP32 DevKit) | Bridge (ESP32-C3) | STM32 Nucleo |
|---|---|---|
| GPIO17 (TX2) | GPIO7 | PA10 = USART1_RX (D2) |
| GPIO16 (RX2) | GPIO6 | PA9 = USART1_TX (D8) |
| GND | GND | GND |
| VIN / 5V | 5V | 5 V from the BEC (not the Nucleo 3V3: Wi-Fi peaks ~300 mA) |

TX goes to RX and RX to TX. Both are 3.3 V logic, so no level shifter is needed. On the bench, USB powers the bridge, but the GND wire to the Nucleo is still required.

## Build & flash

```bash
cd bridge
idf.py set-target esp32       # or esp32c3 (needs: ~/esp/esp-idf-v6.0.3/install.sh esp32c3)
idf.py build
idf.py -p /dev/ttyUSB1 flash monitor
```

The STM32 must be built with `FALCON_RC_RADIO=BRIDGE` (the default) and the controller with the ESP-NOW transport (the default).

## Pairing

Nothing to configure: no MAC addresses in the code.

1. The controller broadcasts until something answers.
2. The bridge locks onto the first controller it hears and replies with a HELLO.
3. The controller locks onto that bridge and switches to unicast, which ESP-NOW acknowledges and retries.

Both stay locked until reboot. Every packet is a framed, CRC-checked `rc_frame.h` frame, so other ESP-NOW traffic on the channel is ignored.

## What the log says

```
I bridge: FalconDrone bridge up — MAC ..., Wi-Fi channel 1, UART1 TX=17 RX=16 @ 115200
W bridge: waiting for the controller (Wi-Fi channel 1)
I bridge: serving controller e4:65:b8:0c:58:34
I bridge: air→stm 100  stm→air 100  (fail 0, dropped 0, uart crc err 0)
```

Counts are per 2 s, so 100 = 50 Hz. `— STM32 is not answering` at the end means packets go out on the UART but nothing comes back: check TX↔RX crossing, GND, and that the STM32 runs the BRIDGE build.
