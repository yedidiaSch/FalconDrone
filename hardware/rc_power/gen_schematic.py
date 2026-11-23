#!/usr/bin/env python3
"""
Generate rc_power.kicad_sch: the handheld RC's power board. It replaces the
TP4056 + MT3608 modules and carries the 18650 itself; it screws to the back
of the RC carrier (hardware/rc_carrier) and plugs in with four pins.

  USB-C (charge only, 5V/GND/CC) -> IP5306: 2.1 A switching charger, 5 V
  boost, 4-LED fuel gauge, power button. Wiring per the Injoinic IP5306
  V1.10 typical application (fig. 7) without the flashlight and the phone
  plug-in detector: 1 uH inductor, 22u on BAT and VOUT, 2R + 10u snubbers
  on VIN and BAT.
  18650 <- DW01A + 8205A in the cell's negative lead (over-discharge,
  overcharge, short circuit), like hardware/controller. Board GND = pack
  minus after the switch; BATT_N only between the cell, the DW01A and the
  FETs.

The IP5306 turns its boost off after 32 s below 45 mA. The RC draws more
than that, but a controller must never switch itself off mid-flight, so the
ESP32 can "press" the KEY pin through Q2 (a short press keeps it on, a long
one turns it off): firmware pulses PWR_KEEP for ~100 ms every ~10 s.

Output header J2 (to the RC carrier): 1 +5V, 2 GND, 3 VBAT (cell voltage,
for the fuel gauge ADC), 4 PWR_KEEP (ESP32 GPIO, 3.3 V logic).

JLCPCB places everything except the 18650 holder (hand-soldered).

Usage:  python3 gen_schematic.py   (then: kicad-cli sch erc rc_power.kicad_sch)
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import schgen  # noqa: E402
from schgen import C, R, custom_symbol, header, init, note, part, write_project_lib, write_sch  # noqa: E402

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT = "rc_power"
init(PROJECT, OUT_DIR)
schgen.R_FP = "Resistor_SMD:R_0805_2012Metric"
schgen.C_FP = "Capacitor_SMD:C_0805_2012Metric"

# IP5306 (Injoinic V1.10 pin table), ESOP-8 with the exposed pad on GND.
IP5306_PINS = [
    ("1", "VIN", "power_in", -12.7, 5.08, 0),
    ("2", "LED1", "bidirectional", -12.7, 0, 0),
    ("3", "LED2", "bidirectional", -12.7, -2.54, 0),
    ("4", "LED3", "bidirectional", -12.7, -5.08, 0),
    ("5", "KEY", "input", 12.7, -5.08, 180),
    ("6", "BAT", "power_in", 12.7, 0, 180),
    ("7", "SW", "power_out", 12.7, 2.54, 180),
    ("8", "VOUT", "power_out", 12.7, 5.08, 180),
    ("9", "GND", "power_in", 0, -10.16, 90),
]
ESOP8 = "Package_SO:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.29x3mm_ThermalVias"

# 8205A dual N-MOSFET, common drain (same table as hardware/controller).
FS8205A_PINS = [
    ("2", "S1", "passive", -10.16, 2.54, 0),
    ("3", "S1", "passive", -10.16, 0, 0),
    ("4", "G1", "input", -10.16, -2.54, 0),
    ("6", "S2", "passive", 10.16, 2.54, 180),
    ("7", "S2", "passive", 10.16, 0, 180),
    ("5", "G2", "input", 10.16, -2.54, 180),
    ("1", "D", "passive", -2.54, 10.16, 270),
    ("8", "D", "passive", 2.54, 10.16, 270),
]


def symbols():
    return [
        custom_symbol("IP5306", "U", ESOP8,
                      "https://www.lcsc.com/datasheet/lcsc_datasheet_2304140030_INJOINIC-IP5306_C181692.pdf",
                      "Li-ion charger 2.1 A + 5 V 2.1 A boost + fuel gauge, ESOP-8",
                      "power bank charger boost", "SOIC*1EP*", IP5306_PINS, half_w=10.16, half_h=7.62),
        custom_symbol("FS8205A", "Q", "Package_SO:TSSOP-8_4.4x3mm_P0.65mm",
                      "https://www.lcsc.com/datasheet/lcsc_datasheet_1810292110_Fortune-Semicon-FS8205A_C32254.pdf",
                      "Dual N-MOSFET 20 V 6 A, common drain, Li-ion protection switch, TSSOP-8",
                      "MOSFET dual N common drain battery protection DW01", "TSSOP*4.4x3mm*P0.65mm*",
                      FS8205A_PINS, half_w=7.62, half_h=7.62),
    ]


def build():
    # ---- USB-C, charge only
    note("USB-C  (charging input, 5 V)", 20, 25)
    part("J1", "Connector", "USB_C_Receptacle_USB2.0_16P", "USB-C", (35, 55), {
        "VBUS": "VIN", "GND": "GND", "SHIELD": "GND", "CC1": "CC1", "CC2": "CC2",
    }, "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12", nc_ok=True)
    R("R1", "5.1k", (60, 70), "CC1", "GND")
    R("R2", "5.1k", (70, 70), "CC2", "GND")
    note("5.1k on CC1/CC2: a USB-C charger sees a sink and turns on 5 V", 50, 82, 1.27)

    # ---- IP5306
    note("IP5306  charger + 5 V boost + fuel gauge", 100, 25)
    part("U1", PROJECT, "IP5306", "IP5306", (140, 60), {
        "VIN": "VIN", "LED1": "LED1", "LED2": "LED2", "LED3": "LED3", "KEY": "KEY",
        "BAT": "BATT_P", "SW": "SW", "VOUT": "+5V", "GND": "GND",
    })
    C("C1", "10u", (105, 45), "VIN")
    R("R3", "2.2", (115, 45), "VIN", "VIN_SNUB")
    C("C2", "10u", (115, 55), "VIN_SNUB")
    part("L1", "Device", "L", "1u", (170, 45), {"1": "SW", "2": "BATT_P"},
         "Inductor_SMD:L_Changjiang_FNR5020S")
    C("C3", "22u", (180, 55), "BATT_P")
    R("R4", "2.2", (190, 45), "BATT_P", "BAT_SNUB")
    C("C4", "10u", (190, 55), "BAT_SNUB")
    for i, x in enumerate((205, 215, 225)):
        C(f"C{5 + i}", "22u", (x, 45), "+5V")
    note("L1 SW -> BAT, short and wide. R3/C2, R4/C4: snubbers from the\n"
         "reference design. Exposed pad = GND, thermal vias into both pours.", 160, 75, 1.27)

    # ---- fuel gauge LEDs: charlieplexed as in fig. 7 (LED3 is the common node)
    note("FUEL GAUGE  (25 / 50 / 75 / 100 %)", 100, 100)
    leds = [("D1", "LED1", "LED3"), ("D2", "LED3", "LED1"), ("D3", "LED2", "LED3"), ("D4", "LED3", "LED2")]
    for i, (ref, a, k) in enumerate(leds):
        part(ref, "Device", "LED", "white", (110 + 15 * i, 120), {"A": a, "K": k},
             "LED_SMD:LED_0603_1608Metric")

    # ---- power button + ESP32 keep-alive
    note("POWER BUTTON  (short: on / show charge, long 2 s: off)", 100, 140)
    part("SW1", "Switch", "SW_Push", "POWER", (115, 160), {"1": "KEY", "2": "GND"},
         "Button_Switch_SMD:SW_Push_1P1T_XKB_TS-1187A")
    part("Q2", "Transistor_FET", "2N7002", "2N7002", (150, 160),
         {"D": "KEY", "G": "PWR_KEEP", "S": "GND"})
    R("R5", "100k", (140, 175), "PWR_KEEP", "GND")
    note("Q2 is the ESP32's finger on the button. R5 keeps it\n"
         "released while the RC board is unplugged or booting.", 130, 185, 1.27)

    # ---- 18650 + protection
    note("18650 + PROTECTION  DW01A / 8205A", 20, 105)
    part("BT1", "Device", "Battery_Cell", "18650", (30, 135), {"+": "BATT_P", "-": "BATT_N"},
         "Battery:BatteryHolder_Keystone_1042_1x18650")
    R("R6", "100", (45, 125), "BATT_P", "DW_VCC")
    C("C8", "100n", (55, 125), "DW_VCC", "BATT_N")
    part("U2", "Battery_Management", "DW01A", "DW01A", (70, 145), {
        "VCC": "DW_VCC", "GND": "BATT_N", "OD": "DW_OD", "OC": "DW_OC", "CS": "DW_CS",
    })
    R("R7", "1k", (85, 160), "DW_CS", "GND")
    part("Q1", PROJECT, "FS8205A", "8205A", (60, 175), {
        "S1": "BATT_N", "G1": "DW_OD", "S2": "GND", "G2": "DW_OC", "D": "DW_DRAIN",
    })
    note("GND = pack minus after Q1. Mark + / - big on the silk:\na reversed cell is not protected.",
         20, 195, 1.27)

    # ---- output to the RC carrier
    note("TO THE RC BOARD", 180, 140)
    header("J2", "TO RC", (190, 165), ["+5V", "GND", "BATT_P", "PWR_KEEP"])

    for i in range(4):
        part(f"H{i + 1}", "Mechanical", "MountingHole", "M3", (230 + 10 * i, 120), {},
             "MountingHole:MountingHole_3.2mm_M3", in_bom=False)
    note("H1-H4: M3, 11 mm standoffs to the RC board", 225, 130, 1.27)

    for i, net in enumerate(("VIN", "BATT_P", "BATT_N", "GND", "DW_VCC")):
        part(f"#FLG0{i + 1}", "power", "PWR_FLAG", "PWR_FLAG", (20 + 10 * i, 15), {"1": net})


if __name__ == "__main__":
    write_project_lib(symbols(), "FalconDrone RC power parts")
    build()
    write_sch("FalconDrone RC power board")
