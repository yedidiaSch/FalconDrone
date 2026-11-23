#!/usr/bin/env python3
"""
Generate falcon_rc_carrier.kicad_sch: a carrier board for the handheld RC,
built from ready-made modules (the simple sibling of hardware/controller).

Modules, all hand-soldered or socketed:
  ESP32 DevKit V1, 30 pins, USB-C   2x 1x15 female sockets, rows 25.4 mm apart
  nRF24L01+ module                  2x4 female socket
  SSD1306/SH1106 OLED, 4 pins       1x4 female socket, GND VCC SCL SDA
  2x KY-023 joystick                flat on M3 screws, 5 short wires / straight header
  ARM, MODE slide switches          3 holes each, 2.54 mm
  RC power board (hardware/rc_power) on the back, 1x4 socket J10 + 4 standoffs
JLCPCB places only the SMD resistors and capacitors (gen_fab.py).

Power: the power board (18650 + protection + IP5306 charger / 5 V boost +
power button) feeds +5V into the DevKit's VIN; the DevKit's own LDO makes
3.3 V for everything else. J10: +5V, GND, VBATT (the cell), PWR_KEEP. The
cell voltage is halved onto GPIO36 (100k/100k: 21 uA, years for a 3 Ah
cell). PWR_KEEP (GPIO17) "presses" the IP5306's button through a FET on the
power board: pulse it ~100 ms every ~10 s, or the IP5306 may switch the
boost off at light load; hold it 2 s to power the RC off.

Mode 2 sticks: left = throttle (Y) / yaw (X), right = pitch (Y) / roll (X).
Pins (ADC1 only, ADC2 is unusable with Wi-Fi on):
  right stick  X GPIO32, Y GPIO33, button GPIO25   (as in controller/main/board.h)
  left stick   X GPIO35, Y GPIO34, button GPIO13   (GPIO34 was the throttle pot)
  ARM GPIO26, MODE GPIO27 (switch middle pin, one end to GND)
  nRF24 VSPI: SCK 18, MOSI 23, MISO 19, CSN 5, CE 4;  OLED I2C: SDA 21, SCL 22
  VBAT_SENSE GPIO36 (VP)
  PWR_KEEP GPIO17
New for the firmware: GPIO35 yaw, GPIO13 left button, GPIO36 battery,
GPIO17 power keep-alive.

Usage:  python3 gen_schematic.py   (then: kicad-cli sch erc falcon_rc_carrier.kicad_sch)
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import schgen  # noqa: E402
from schgen import C, R, header, init, note, part, write_sch  # noqa: E402

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT = "falcon_rc_carrier"
init(PROJECT, OUT_DIR)

# SMD, placed by JLCPCB (hand-solder pads: easy to rework)
schgen.R_FP = "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder"
schgen.C_FP = "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder"

# DevKit pins, top to bottom with USB-C at the top (as printed on the board).
# The footprint is a plain 1x15 pin header: same positions as the DevKit seen
# from the top, which is what a female socket under it needs.
ESP_LEFT = ["+3V3", "GND", None, None, "NRF_CE", None, "PWR_KEEP", "NRF_CSN",
            "NRF_SCK", "NRF_MISO", "OLED_SDA", None, None, "OLED_SCL", "NRF_MOSI"]
#           3V3     GND    D15   D2    D4        D16   D17   D5
#           D18        D19         D21         RX0   TX0   D22         D23
ESP_RIGHT = ["+5V", "GND", "STK_L_SW", None, None, "MODE_SW", "ARM_SW", "STK_R_SW",
             "STK_R_Y", "STK_R_X", "STK_L_X", "STK_L_Y", None, "VBAT_SENSE", None]
#            VIN    GND    D13         D12   D14   D27        D26       D25
#            D33        D32        D35        D34        VN    VP            EN
HDR = "Connector_PinHeader_2.54mm:PinHeader_1x{:02d}_P2.54mm_Vertical"


def esp_column(ref, value, at, nets):
    part(ref, "Connector_Generic", "Conn_01x15", value, at,
         {str(i + 1): n for i, n in enumerate(nets) if n}, HDR.format(15), nc_ok=True)


def build():
    # ---- ESP32
    note("ESP32 DevKit V1 (30 pins, USB-C at the top edge)", 20, 25)
    esp_column("J1", "ESP left socket", (40, 70), ESP_LEFT)
    esp_column("J2", "ESP right socket", (80, 70), ESP_RIGHT)
    note("J1: 3V3 GND D15 D2 D4 D16 D17 D5 D18 D19 D21 RX0 TX0 D22 D23\n"
         "J2: VIN GND D13 D12 D14 D27 D26 D25 D33 D32 D35 D34 VN VP EN",
         20, 100, 1.27)
    C("C2", "22u", (100, 55), "+5V")
    note("C2 at VIN", 95, 65, 1.27)

    # ---- Power
    note("POWER  (from the RC power board on the back)", 130, 25)
    header("J10", "POWER BOARD", (140, 50), ["+5V", "GND", "VBATT", "PWR_KEEP"])
    note("1x4 female socket on the back, 11 mm M3 standoffs (H13-H16)", 130, 62, 1.27)
    for i, net in enumerate(("+5V", "+3V3", "GND", "VBATT")):
        part(f"#FLG0{i + 1}", "power", "PWR_FLAG", "PWR_FLAG", (135 + 12 * i, 35), {"1": net})

    note("BATTERY SENSE  (4.2 V -> 2.1 V on GPIO36)", 130, 85)
    R("R1", "100k", (140, 100), "VBATT", "VBAT_SENSE")
    R("R2", "100k", (152, 100), "VBAT_SENSE", "GND")
    C("C1", "100n", (164, 100), "VBAT_SENSE")

    # ---- Radio + display
    note("RADIO  nRF24L01+ (VSPI)", 20, 125)
    part("J6", "RF", "NRF24L01_Breakout", "nRF24L01+", (45, 150), {
        "VCC": "+3V3", "GND": "GND", "CE": "NRF_CE", "~{CSN}": "NRF_CSN",
        "SCK": "NRF_SCK", "MOSI": "NRF_MOSI", "MISO": "NRF_MISO",
    }, nc_ok=True)                     # IRQ: the driver polls
    C("C3", "10u", (75, 145), "+3V3")
    C("C4", "100n", (85, 145), "+3V3")
    note("2x4 female socket; C3/C4 at the module", 65, 160, 1.27)

    note("OLED  (I2C 0x3C)", 110, 125)
    header("J7", "OLED", (115, 150), ["GND", "+3V3", "OLED_SCL", "OLED_SDA"])
    C("C5", "100n", (130, 145), "+3V3")

    # ---- Sticks + switches
    note("STICKS  (KY-023, powered from 3.3 V)", 150, 125)
    header("J8", "LEFT STICK", (160, 150), ["GND", "+3V3", "STK_L_X", "STK_L_Y", "STK_L_SW"])
    header("J9", "RIGHT STICK", (185, 150), ["GND", "+3V3", "STK_R_X", "STK_R_Y", "STK_R_SW"])
    note("pins as on the module: GND +5V VRx VRy SW\n(+5V pin gets 3.3 V)", 150, 165, 1.27)

    note("SWITCHES  (middle pin to the GPIO)", 20, 185)
    part("SW2", "Connector_Generic", "Conn_01x03", "ARM", (30, 205),
         {"1": "GND", "2": "ARM_SW"}, HDR.format(3), nc_ok=True)
    part("SW3", "Connector_Generic", "Conn_01x03", "MODE", (55, 205),
         {"1": "GND", "2": "MODE_SW"}, HDR.format(3), nc_ok=True)

    # ---- Mounting
    for i in range(4):
        part(f"H{i + 1}", "Mechanical", "MountingHole", "M3", (100 + 12 * i, 200), {},
             "MountingHole:MountingHole_3.2mm_M3", in_bom=False)
    for i in range(8):
        part(f"H{i + 5}", "Mechanical", "MountingHole", "M3 stick", (100 + 12 * (i % 4), 215 + 12 * (i // 4)),
             {}, "MountingHole:MountingHole_3.5mm", in_bom=False)
    for i in range(4):
        part(f"H{i + 13}", "Mechanical", "MountingHole", "M3 power", (100 + 12 * i, 240), {},
             "MountingHole:MountingHole_3.2mm_M3", in_bom=False)
    note("H1-H4 corners, H5-H12 joystick screws (3.5 mm: forgives ~0.5 mm),\n"
         "H13-H16 power board standoffs", 95, 252, 1.27)


if __name__ == "__main__":
    build()
    write_sch("FalconDrone RC carrier")
