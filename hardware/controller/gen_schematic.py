#!/usr/bin/env python3
"""
Generate falcon_rc.kicad_sch: the FalconDrone handheld controller schematic.

Gamepad-style board, mode 2: left stick throttle (Y) / yaw (X), right stick
pitch (Y) / roll (X). ESP32-S3 (native USB: flashing, console and charging
all on one USB-C, no USB-UART bridge), nRF24L01+ module as the RC link
(the flight-controller board carries only an nRF24), SSD1306 OLED module,
one 18650 cell on the back. Sticks are bare Alps RKJXV1224005 (PS2 type).

Power path:
  USB-C VBUS -> BQ24073 (charger + power path, 0.74 A charge, 1.07 A input limit, 9 h timer)
       BQ24073 OUT = VSYS: 4.4 V on USB, battery voltage otherwise, so the
       controller runs while charging and charges while switched off.
  18650 <- DW01A + 8205A protection (over-discharge ~2.5 V, overcharge,
       short circuit) in the cell's negative lead; board GND is pack minus.
  VSYS -> AP2112K-3.3 (600 mA LDO) -> +3V3. The power switch drives the LDO
       enable, so it carries no load current. Below ~3.5 V cell the LDO
       drops out: firmware warns at 3.6 V and shuts down at 3.4 V.
  +3V3 -> ferrite -> +3V3A for the stick pots. VPOT_SENSE reads that rail
       so the firmware can divide it out (ratiometric reading): rail ripple
       from radio bursts then no longer moves the stick values.

ESP32-S3-WROOM-1 pin map (N8 or N16, any 3.3 V-flash variant: IO35-37 are
left free so an octal-PSRAM module also works; not the 1.8 V "...V" variants,
where IO47/IO48 run at 1.8 V; straps IO45/IO46 unused):
  ADC1 (ADC2 conflicts with Wi-Fi)
    IO1  STICK_LX (yaw)      IO2  STICK_LY (throttle)
    IO4  STICK_RX (roll)     IO5  STICK_RY (pitch)
    IO6  VBAT_SENSE (1:2)    IO7  VPOT_SENSE (1:2)
  OLED  I2C   IO8 SDA / IO9 SCL, SSD1306 0x3C
  nRF24 FSPI  IO10 CSN / IO11 MOSI / IO12 SCK / IO13 MISO (IO-MUX pins), IO14 CE, IO21 IRQ
  Buttons (active low, internal pull-ups)
    IO15 BTN_L (left stick press)   IO16 BTN_R (right stick press)
    IO17 BTN_UP  IO18 BTN_DOWN  IO38 BTN_OK  IO39 BTN_BACK
  Switches (to GND, internal pull-ups)
    IO40 SW_ARM    IO41 SW_MODE_A / IO42 SW_MODE_B (3-position, centre = neither)
  IO47 BUZZER (NPN driver)   IO48 LED_STATUS   IO3 VBUS_DET (USB present)
  IO0 BOOT button   IO19/IO20 USB D-/D+   TXD0/RXD0 debug header

Usage:  python3 gen_schematic.py   (then: kicad-cli sch erc falcon_rc.kicad_sch)
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from schgen import (C, R, custom_symbol, header, init, note, part,  # noqa: E402
                    write_project_lib, write_sch)

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT = "falcon_rc"
init(PROJECT, OUT_DIR)

# FS8205A dual N-MOSFET, common drain (the usual DW01A partner), TSSOP-8.
# Fortune's part is out of stock at JLCPCB; the BOM fits HXY 8205A (C5148694),
# same TSSOP-8 pinout per its datasheet.
# Pinout per the Fortune Semiconductor datasheet: 1/8 drain, 2/3 S1, 4 G1,
# 5 G2, 6/7 S2. (number, name, electrical type, x, y, rotation)
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


def fs8205a_symbol():
    return custom_symbol(
        "FS8205A", "Q", "Package_SO:TSSOP-8_4.4x3mm_P0.65mm",
        "https://www.lcsc.com/datasheet/lcsc_datasheet_1810292110_Fortune-Semicon-FS8205A_C32254.pdf",
        "Dual N-MOSFET 20 V 6 A, common drain, Li-ion protection switch, TSSOP-8",
        "MOSFET dual N common drain battery protection DW01", "TSSOP*4.4x3mm*P0.65mm*",
        FS8205A_PINS, half_w=7.62, half_h=7.62)


# Alps RKJXV1224005 thumb stick: two 10k pots + push switch (pads: see
# gen_footprints.py). VR1 sits on the side and turns when the stick tilts
# forward/back (Y), VR2 on the top edge for left/right (X).
JOYSTICK_PINS = [
    ("1", "VR1_1", "passive", -12.7, 7.62, 0),
    ("2", "VR1_W", "passive", -12.7, 5.08, 0),
    ("3", "VR1_3", "passive", -12.7, 2.54, 0),
    ("4", "VR2_1", "passive", -12.7, -2.54, 0),
    ("5", "VR2_W", "passive", -12.7, -5.08, 0),
    ("6", "VR2_3", "passive", -12.7, -7.62, 0),
    ("7", "SW_a", "passive", 12.7, 5.08, 180),
    ("8", "SW_c", "passive", 12.7, 2.54, 180),
    ("9", "SW_b", "passive", 12.7, -2.54, 180),
    ("10", "SW_d", "passive", 12.7, -5.08, 180),
    ("MP", "FRAME", "passive", 0, -12.7, 90),
]


def joystick_symbol():
    return custom_symbol(
        "RKJXV1224005", "JS", "falcon_rc:Joystick_Alps_RKJXV122400x",
        "https://tech.alpsalpine.com/e/products/detail/RKJXV1224005/",
        "Thumb joystick, 2 x 10k pot (VR1 = Y, VR2 = X) + push switch (a-c / b-d)",
        "joystick thumbstick potentiometer Alps", "Joystick_Alps_RKJXV*",
        JOYSTICK_PINS)


def joystick(ref, value, at, x_net, y_net, btn_net):
    part(ref, PROJECT, "RKJXV1224005", value, at, {
        "VR1_1": "+3V3A", "VR1_W": y_net, "VR1_3": "GND",
        "VR2_1": "+3V3A", "VR2_W": x_net, "VR2_3": "GND",
        "SW_a": btn_net, "SW_c": btn_net, "SW_b": "GND", "SW_d": "GND", "FRAME": "GND",
    })


SW_TACT_THT = "Button_Switch_THT:SW_PUSH_6mm_H5mm"
TOGGLE_FP = "falcon_rc:Toggle_SPDT_P4.70mm"
SW_TACT_SMD = "Button_Switch_SMD:SW_SPST_PTS810"
# Through-hole LEDs: every SMD part sits on the back, so the front (controls
# only) needs no assembly pass.
LED_FP = "LED_THT:LED_D3.0mm"


def build():
    # ---- USB-C in
    note("USB-C  (charge + flash + console)", 20, 25)
    part("J1", "Connector", "USB_C_Receptacle_USB2.0_16P", "USB-C", (40, 60), {
        "VBUS": "VBUS", "GND": "GND", "SHIELD": "GND",
        "CC1": "CC1", "CC2": "CC2", "D+": "USB_DP", "D-": "USB_DN",
    }, "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12", nc_ok=True)   # SBU1/2 open
    R("R1", "5.1k", (70, 80), "CC1", "GND")
    R("R2", "5.1k", (80, 80), "CC2", "GND")
    note("R1/R2: sink Rd, the source then gives 5 V\n(no Rd = a C-to-C charger stays off)", 60, 95, 1.27)
    part("U1", "Power_Protection", "USBLC6-2SC6", "USBLC6-2SC6", (100, 55), {
        "I/O1": "USB_DN", "I/O2": "USB_DP", "VBUS": "VBUS", "GND": "GND",
    })
    note("U1 at the connector; D+/D- 90 ohm pair\nstraight to the module, no stubs", 90, 72, 1.27)
    part("#FLG01", "power", "PWR_FLAG", "PWR_FLAG", (60, 30), {"1": "VBUS"})
    part("#FLG02", "power", "PWR_FLAG", "PWR_FLAG", (70, 30), {"1": "GND"})

    # ---- Charger + power path
    # BQ24073, not the BQ24074 of the first draft: JLCPCB stocks it and the
    # two differ only in OVP (6.6 V vs 10.5 V, fine on USB) and pin 15
    # (TD instead of ITERM). Same 4.4 V OUT, same ISET/ILIM/TMR constants.
    note("CHARGER  BQ24073 (power path)", 130, 25)
    part("U2", "Battery_Management", "BQ24073RGT", "BQ24073RGT", (165, 60), {
        "IN": "VBUS", "OUT": "VSYS", "BAT": "BATT_P", "VSS": "GND",
        "ISET": "CHG_ISET", "ILIM": "CHG_ILIM", "TS": "CHG_TS",
        "~{CE}": "GND", "EN1": "GND", "EN2": "VSYS",
        "~{PGOOD}": "LED_PG_K", "~{CHG}": "LED_CHG_K", "TMR": "CHG_TMR",
        "TD": "GND",   # termination on (10 % of ISET), datasheet table 7-1
    })
    C("C1", "1u", (135, 45), "VBUS")
    C("C2", "10u", (200, 45), "VSYS", bulk=True)
    C("C3", "4.7u", (210, 45), "BATT_P", bulk=True)
    R("R3", "1.2k", (135, 90), "CHG_ISET", "GND")
    R("R4", "1.5k", (145, 90), "CHG_ILIM", "GND")
    R("R5", "10k", (155, 90), "CHG_TS", "GND")
    R("R24", "68k", (165, 90), "CHG_TMR", "GND")
    note("Values per TI SLUS810N:\n"
         "R3 ISET 1.2k: charge 890 A-ohm / 1.2k = 0.74 A (0.66-0.81)\n"
         "R4 ILIM 1.5k: input limit 1610 / 1.5k = 1.07 A (EN2=1 EN1=0)\n"
         "R24 TMR 68k: fast-charge timer 10 x 48 s/k x 68k = 9.1 h\n"
         "(6.8 h min; the 5 h default can stop a 3.5 Ah cell early)\n"
         "R5 TS: fixed 10k, temperature check off (no NTC in the\n"
         "holder). Swap for a 10k NTC on the cell to enable it.\n"
         "TD low = termination on at 10 %. Thermal pad to GND with\n"
         "vias: ~1 W at 3.6 V cell, TJ(REG) 125 C throttles beyond.", 130, 117, 1.27)
    R("R6", "1k", (200, 75), "VSYS", "LED_PG_A")
    part("D1", "Device", "LED", "green PWR", (215, 75), {"A": "LED_PG_A", "K": "LED_PG_K"}, LED_FP)
    R("R7", "1k", (200, 98), "VSYS", "LED_CHG_A")
    part("D2", "Device", "LED", "red CHG", (215, 98), {"A": "LED_CHG_A", "K": "LED_CHG_K"}, LED_FP)
    note("LEDs from VSYS: they work with the\npower switch off", 195, 113, 1.27)

    # ---- Battery + protection
    note("18650 + PROTECTION  DW01A / FS8205A", 20, 120)
    part("BT1", "Device", "Battery_Cell", "18650", (35, 150), {"+": "BATT_P", "-": "BATT_N"},
         "Battery:BatteryHolder_Keystone_1042_1x18650")
    R("R8", "100", (55, 140), "BATT_P", "DW_VCC")
    C("C4", "100n", (65, 140), "DW_VCC", "BATT_N")
    part("U3", "Battery_Management", "DW01A", "DW01A", (85, 155), {
        "VCC": "DW_VCC", "GND": "BATT_N", "OD": "DW_OD", "OC": "DW_OC", "CS": "DW_CS",
    })
    R("R9", "1k", (105, 170), "DW_CS", "GND")       # DW01A typical application value
    part("Q1", PROJECT, "FS8205A", "FS8205A", (125, 162), {
        "S1": "BATT_N", "G1": "DW_OD", "S2": "GND", "G2": "DW_OC", "D": "DW_DRAIN",
    })
    part("#FLG03", "power", "PWR_FLAG", "PWR_FLAG", (150, 155), {"1": "BATT_N"})
    part("#FLG04", "power", "PWR_FLAG", "PWR_FLAG", (160, 155), {"1": "DW_VCC"})
    note("Board GND = pack minus (after Q1). BATT_N only\n"
         "between the cell, U3 and Q1. Keystone 1042 on\n"
         "the back; mark + / - polarity big on the silk:\n"
         "a reversed cell is not protected.", 20, 180, 1.27)

    # ---- 3V3
    note("3V3  AP2112K + POWER SWITCH", 230, 25)
    part("U4", "Regulator_Linear", "AP2112K-3.3", "AP2112K-3.3", (265, 55), {
        "VIN": "VSYS", "GND": "GND", "EN": "LDO_EN", "VOUT": "+3V3",
    })
    C("C5", "10u", (240, 50), "VSYS", bulk=True)
    C("C6", "10u", (290, 50), "+3V3", bulk=True)
    C("C7", "100n", (300, 50), "+3V3")
    part("SW1", "Switch", "SW_SPDT", "POWER", (250, 85),
         {"B": "LDO_EN", "A": "VSYS", "C": "GND"}, "Button_Switch_SMD:SW_SPDT_PCM12")
    R("R10", "100k", (270, 85), "LDO_EN", "GND")
    note("SW1 slide switch on the LDO enable,\nno load current through it", 240, 98, 1.27)

    note("STICK SUPPLY + SENSE", 230, 115)
    part("FB1", "Device", "FerriteBead_Small", "600R@100MHz", (238, 130),
         {"1": "+3V3", "2": "+3V3A"}, "Inductor_SMD:L_0603_1608Metric")
    part("#FLG05", "power", "PWR_FLAG", "PWR_FLAG", (248, 120), {"1": "+3V3A"})
    C("C8", "10u", (250, 135), "+3V3A", bulk=True)
    C("C9", "100n", (260, 135), "+3V3A")
    R("R11", "100k", (272, 132), "+3V3A", "VPOT_SENSE")
    R("R12", "100k", (282, 132), "VPOT_SENSE", "GND")
    C("C10", "100n", (292, 132), "VPOT_SENSE")
    R("R13", "220k", (304, 132), "BATT_P", "VBAT_SENSE")
    R("R14", "220k", (314, 132), "VBAT_SENSE", "GND")
    C("C11", "100n", (324, 132), "VBAT_SENSE")
    R("R15", "100k", (272, 162), "VBUS", "VBUS_DET")
    R("R16", "150k", (282, 162), "VBUS_DET", "GND")
    C("C12", "10n", (292, 162), "VBUS_DET")
    note("VPOT 3.3 -> 1.65 V, VBAT 4.2 -> 2.1 V (~10 uA),\n"
         "VBUS 5 -> 3.0 V. Read with 12 dB attenuation.", 235, 178, 1.27)

    # ---- MCU
    note("MCU  ESP32-S3-WROOM-1", 130, 195)
    part("U5", "RF_Module", "ESP32-S3-WROOM-1", "ESP32-S3-WROOM-1-N8", (170, 240), {
        "3V3": "+3V3", "GND": "GND", "EN": "ESP_EN", "IO0": "BOOT",
        "USB_D-": "USB_DN", "USB_D+": "USB_DP", "TXD0": "UART_TX", "RXD0": "UART_RX",
        "IO1": "STICK_LX", "IO2": "STICK_LY", "IO4": "STICK_RX", "IO5": "STICK_RY",
        "IO6": "VBAT_SENSE", "IO7": "VPOT_SENSE", "IO3": "VBUS_DET",
        "IO8": "I2C_SDA", "IO9": "I2C_SCL",
        "IO10": "NRF_CSN", "IO11": "NRF_MOSI", "IO12": "NRF_SCK", "IO13": "NRF_MISO",
        "IO14": "NRF_CE", "IO21": "NRF_IRQ",
        "IO15": "BTN_L", "IO16": "BTN_R",
        "IO17": "BTN_UP", "IO18": "BTN_DOWN", "IO38": "BTN_OK", "IO39": "BTN_BACK",
        "IO40": "SW_ARM", "IO41": "SW_MODE_A", "IO42": "SW_MODE_B",
        "IO47": "BUZZER", "IO48": "LED_STATUS",
    }, nc_ok=True)   # IO35-37 free (taken by octal PSRAM on R8 modules), IO45/46 straps
    C("C13", "10u", (215, 205), "+3V3", bulk=True)
    C("C14", "100n", (225, 205), "+3V3")
    note("C13/C14 at module pin 2. Antenna end over\nthe board edge, no copper under it (keepout).",
         205, 218, 1.27)
    R("R17", "10k", (215, 235), "+3V3", "ESP_EN")
    C("C15", "1u", (225, 235), "ESP_EN")
    part("SW2", "Switch", "SW_Push", "RESET", (240, 235), {"1": "ESP_EN", "2": "GND"}, SW_TACT_SMD)
    R("R18", "10k", (215, 256), "+3V3", "BOOT")
    part("SW3", "Switch", "SW_Push", "BOOT", (240, 256), {"1": "BOOT", "2": "GND"}, SW_TACT_SMD)
    note("C15 delays EN until 3V3 is up. Native USB normally\n"
         "auto-resets into download; hold BOOT + tap RESET\nif the firmware has broken USB.",
         205, 268, 1.27)
    # not assembled: native USB carries the console; solder pins if needed
    header("J2", "DEBUG_UART", (295, 240), ["GND", "UART_TX", "UART_RX", "+3V3"], dnp=True)

    # ---- Radio
    note("RADIO  nRF24L01+ (PA+LNA) module", 20, 195)
    part("J3", "RF", "NRF24L01_Breakout", "nRF24L01+", (50, 225), {
        "VCC": "+3V3", "GND": "GND", "CE": "NRF_CE", "~{CSN}": "NRF_CSN",
        "SCK": "NRF_SCK", "MOSI": "NRF_MOSI", "MISO": "NRF_MISO", "IRQ": "NRF_IRQ",
    })
    C("C16", "22u", (85, 220), "+3V3", bulk=True)
    C("C17", "100n", (95, 220), "+3V3")
    note("C16/C17 at the socket. Module antenna\n(SMA) above the top edge, opposite\nend from the ESP32 antenna", 75, 233, 1.27)

    # ---- Display
    note("OLED  SSD1306 128x64 module (I2C 0x3C)", 20, 255)
    header("J4", "OLED", (35, 275), ["GND", "+3V3", "I2C_SCL", "I2C_SDA"],
           footprint="Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical")
    R("R19", "4.7k", (60, 270), "+3V3", "I2C_SCL")
    R("R20", "4.7k", (70, 270), "+3V3", "I2C_SDA")
    note("Pin order GND VCC SCL SDA (most 0.96\" modules;\nsome swap GND/VCC, check before soldering)",
         20, 290, 1.27)

    # ---- Sticks
    note("STICKS  Alps RKJXV1224005", 330, 25)
    joystick("JS1", "STICK_LEFT", (350, 45), "STICK_LX", "STICK_LY", "BTN_L")
    joystick("JS2", "STICK_RIGHT", (350, 75), "STICK_RX", "STICK_RY", "BTN_R")
    for i, (net, x) in enumerate((("STICK_LX", 365), ("STICK_LY", 375),
                                   ("STICK_RX", 385), ("STICK_RY", 395))):
        C(f"C{18 + i}", "100n", (x, 50), net)
    note("Pots across +3V3A; frame tabs to GND. Direction\n"
         "per axis is a firmware setting. C18-C21 at the\n"
         "ESP32 pins: charge reservoir for the ADC sample cap.\n"
         "Mode 2: left Y throttle, left X yaw, right X roll,\n"
         "right Y pitch. Left Y self-centres; remove its\n"
         "spring for a throttle that stays put.", 330, 98, 1.27)

    # ---- Buttons + switches
    note("BUTTONS + SWITCHES", 330, 118)
    for i, (net, label_) in enumerate((("BTN_UP", "UP"), ("BTN_DOWN", "DOWN"),
                                       ("BTN_OK", "OK"), ("BTN_BACK", "BACK"))):
        part(f"SW{4 + i}", "Switch", "SW_Push", label_, (345, 130 + 10 * i),
             {"1": net, "2": "GND"}, SW_TACT_THT)
    part("SW8", "Switch", "SW_SPDT", "7101 ARM", (385, 135), {"B": "SW_ARM", "A": "GND"},
         TOGGLE_FP, nc_ok=True)
    part("SW9", "Switch", "SW_SPDT_MSM", "7103 MODE", (385, 155),
         {"2": "GND", "1": "SW_MODE_A", "3": "SW_MODE_B"},
         TOGGLE_FP)
    note("SW4-7 menu / trims. SW8 ARM toggle (ON-ON),\n"
         "SW9 MODE toggle (ON-OFF-ON: ANGLE / RATE /\n"
         "spare). Top-face toggles, PC pins at 4.7 mm:\n"
         "C&K 7101SYCQE / 7103SYCQE (MTS-102/103 fit too).", 330, 172, 1.27)

    # ---- Buzzer + LED
    note("BUZZER + STATUS LED", 330, 190)
    R("R21", "1k", (340, 210), "BUZZER", "BUZ_B")
    part("Q2", "Transistor_BJT", "MMBT3904", "MMBT3904", (355, 210),
         {"B": "BUZ_B", "C": "BUZ_K", "E": "GND"})
    part("BZ1", "Device", "Buzzer", "CPT-9019S", (375, 200), {"+": "BUZ_A", "-": "BUZ_K"},
         "Buzzer_Beeper:Buzzer_CUI_CPT-9019S-SMT")
    R("R22", "22", (390, 197), "+3V3", "BUZ_A")
    R("R25", "1k", (395, 212), "BUZ_A", "BUZ_K")
    note("Piezo transducer (12 nF), PWM 4 kHz (LEDC).\n"
         "R25 discharges it while Q2 is off.\n"
         "Low battery, link lost, armed beeps.", 330, 225, 1.27)
    R("R23", "1k", (340, 240), "LED_STATUS", "LED_ST_A")
    part("D4", "Device", "LED", "yellow", (355, 240), {"A": "LED_ST_A", "K": "GND"}, LED_FP)

    # ---- Mounting
    for i, x in enumerate((95, 105, 115, 125)):
        part(f"H{i + 1}", "Mechanical", "MountingHole", "M3", (x, 270), {},
             "MountingHole:MountingHole_3.2mm_M3", in_bom=False)


if __name__ == "__main__":
    write_project_lib([fs8205a_symbol(), joystick_symbol()], "FalconDrone controller parts")
    build()
    write_sch("FalconDrone controller")
