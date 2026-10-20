#!/usr/bin/env python3
"""
Generate falcon_fc.kicad_sch: the FalconDrone flight-controller schematic.

The circuit is described in Python below and written out as a KiCad 9
schematic. Every pin is connected with a net label placed on the pin end
(no wires), so connectivity is exact and the file stays diff-friendly.
Symbols are copied from the stock KiCad library, flattened if derived.
Parts missing from it (the ICM-42688-P) are defined below and also written
to the project library falcon_fc.kicad_sym.

Pin assignment follows falconDrone.ioc where it can:
  ESC M1..M4      PA0 PA1 (TIM2 CH1/CH2)  PA6 PA7 (TIM3 CH1/CH2)
  nRF24L01        SPI2  PB13 SCK / PB14 MISO / PB15 MOSI, CSN PB12, CE PB1
  Telemetry       USART2 PA2 TX / PA3 RX
  GPS SAM-M10Q    USART6 PC6 TX / PC7 RX (AF8), own LDO (+3V3_GPS)
  AUX header      USART1 PA9 TX / PA10 RX + I2C3 PA8 SCL / PC9 SDA (AF4), Pixhawk GPS pinout
  SWD             PA13 SWDIO / PA14 SWCLK / PB3 SWO
  Status LED      PA5      Calibrate button  PC13
New pins, not yet in the .ioc:
  ICM-42688-P     SPI3  PC10 SCK / PC11 MISO / PC12 MOSI (AF6), CS PA15, INT1 PB5
  MS5611 baro     I2C1  PB6 SCL / PB7 SDA (same bus the MPU-6050 used), addr 0x77
  PA4 VBAT_SENSE (ADC), PC4 NRF_IRQ.
The RC link is the nRF24 only (build with FALCON_RC_RADIO=NRF24); there is
no ESP32 bridge header. The IMU and barometer run from their own low-noise LDO (+3V3_SENS).

Usage:  python3 gen_schematic.py   (then: kicad-cli sch erc falcon_fc.kicad_sch)
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from schgen import (C, R, Q, custom_symbol, find, find1, header, init,  # noqa: E402,F401
                    note, parse_sexp, part, write_project_lib, write_sch)

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT = "falcon_fc"
init(PROJECT, OUT_DIR)


# ---------------------------------------------------------------- project library

# ICM-42688-P, from TDK DS-000347 rev 1.9, table 10 / figure 5. The LGA-14
# 2.5x3.0 mm pad order (1-4 down the left, counter-clockwise) matches KiCad's
# LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y land pattern.
# Pin names are the SPI-mode functions (the datasheet names are multi-function,
# e.g. pin 1 AP_SDO/AP_AD0); this board only uses SPI.
# (number, name, electrical type, x, y, rotation) in symbol units (mm, y up).
ICM42688_PINS = [
    ("12", "AP_CS", "input", -12.7, 5.08, 0),
    ("13", "AP_SCLK", "input", -12.7, 2.54, 0),
    ("14", "AP_SDI", "bidirectional", -12.7, 0, 0),
    ("1", "AP_SDO", "tri_state", -12.7, -2.54, 0),
    ("4", "INT1", "output", 12.7, 2.54, 180),
    ("9", "INT2/FSYNC", "input", 12.7, -2.54, 180),   # grounded FSYNC here
    ("8", "VDD", "power_in", -2.54, 12.7, 270),
    ("5", "VDDIO", "power_in", 2.54, 12.7, 270),
    ("6", "GND", "power_in", -7.62, -12.7, 90),
    ("7", "RESV", "passive", -5.08, -12.7, 90),
    ("2", "RESV", "passive", -2.54, -12.7, 90),
    ("3", "RESV", "passive", 0, -12.7, 90),
    ("10", "RESV", "passive", 2.54, -12.7, 90),
    ("11", "RESV", "passive", 5.08, -12.7, 90),
]


def icm42688_symbol():
    return custom_symbol(
        "ICM-42688-P", "U", "Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y",
        "https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/",
        "6-axis IMU, gyro + accel, SPI 24 MHz / I2C / I3C, LGA-14 2.5x3.0 mm",
        "IMU gyroscope accelerometer TDK InvenSense", "LGA*3x2.5mm*P0.5mm*",
        ICM42688_PINS)


# ---------------------------------------------------------------- the circuit

def build():
    # ---- Power: battery sense + 5 V in (ESC BEC / PDB) -> 3.3 V LDO
    note("POWER", 20, 25)
    # Keyed connector, GND between VBAT and 5V: no plug or solder bridge can put
    # battery voltage on the 5 V rail.
    header("J1", "POWER_IN", (25, 50), ["VBAT", "GND", "+5V_IN"], jst_gh=True)
    note("JST-GH, keyed\n1 VBAT (battery +, max 6S)\n2 GND\n3 +5V from ESC BEC / PDB", 20, 66, 1.27)
    # Reverse-polarity: P-FET with body diode pointing in, gate at GND. Turns on
    # (~50 mOhm) when +5V_IN is positive, blocks when reversed.
    part("Q1", "Transistor_FET", "AO3401A", "AO3401A", (42, 52),
         {"D": "+5V_IN", "S": "+5V", "G": "GND"})
    # Unidirectional TVS. Symbol pin 1 is the cathode (bar), as is D_SMA pad 1.
    part("D2", "Diode", "SMAJ5.0A", "SMAJ5.0A", (57, 64), {"1": "+5V", "2": "GND"})
    note("Q1 reverse-polarity FET, D2 5 V TVS", 20, 80, 1.27)
    part("U2", "Regulator_Linear", "AP7361C-33E", "AP7361C-33E", (85, 50),
         {"VI": "+5V", "GND": "GND", "VO": "+3V3"})
    C("C1", "10u", (68, 50), "+5V", bulk=True)
    C("C2", "10u", (102, 50), "+3V3", bulk=True)
    note("U2 SOT-223: ~0.3 W worst case, give the\ntab (GND) a copper pour, ~+35 C rise", 70, 66, 1.27)
    part("#FLG01", "power", "PWR_FLAG", "PWR_FLAG", (50, 35), {"1": "+5V"})
    part("#FLG02", "power", "PWR_FLAG", "PWR_FLAG", (60, 35), {"1": "GND"})

    note("SENSOR LDO", 100, 90)
    part("U4", "Regulator_Linear", "LP5907MFX-3.3", "LP5907MFX-3.3", (120, 105),
         {"IN": "+5V", "EN": "+5V", "GND": "GND", "OUT": "+3V3_SENS"}, nc_ok=True)
    C("C20", "1u", (102, 105), "+5V")
    C("C21", "1u", (138, 105), "+3V3_SENS")

    note("BATTERY SENSE  (6S 25.2 V -> 2.29 V)", 20, 90)
    note("quiet rail for IMU + baro", 100, 120, 1.27)
    R("R1", "10k", (35, 105), "VBAT", "VBAT_SENSE")
    R("R2", "1k", (50, 105), "VBAT_SENSE", "GND")
    C("C3", "100n", (65, 105), "VBAT_SENSE")

    # ---- MCU
    note("MCU  STM32F446RET6", 150, 25)
    part("U1", "MCU_ST_STM32F4", "STM32F446RETx", "STM32F446RET6", (190, 100), {
        "VDD": "+3V3", "VBAT": "+3V3", "VDDA": "+3V3A",
        "VSS": "GND", "VSSA": "GND", "VCAP_1": "VCAP1",
        "NRST": "NRST", "BOOT0": "BOOT0",
        "PA0": "M1", "PA1": "M2", "PA6": "M3", "PA7": "M4",
        "PA2": "TELEM_TX", "PA3": "TELEM_RX",
        "PA9": "UART1_TX", "PA10": "UART1_RX",
        "PA8": "I2C3_SCL", "PC9": "I2C3_SDA",
        "PC6": "UART6_TX", "PC7": "UART6_RX",
        "PA4": "VBAT_SENSE", "PA5": "LED_STATUS",
        "PA13": "SWDIO", "PA14": "SWCLK", "PB3": "SWO",
        "PC10": "SPI3_SCK", "PC11": "SPI3_MISO", "PC12": "SPI3_MOSI",
        "PA15": "IMU_CS", "PB5": "IMU_INT",
        "PB6": "I2C1_SCL", "PB7": "I2C1_SDA",
        "PB12": "NRF_CSN", "PB1": "NRF_CE", "PC4": "NRF_IRQ",
        "PB13": "SPI2_SCK", "PB14": "SPI2_MISO", "PB15": "SPI2_MOSI",
        "PC13": "BTN_CAL",
    }, nc_ok=True)

    note("MCU DECOUPLING", 240, 25)
    for i, x in enumerate((245, 255, 265, 275)):
        C(f"C{4 + i}", "100n", (x, 40), "+3V3")
    C("C8", "4.7u", (285, 40), "+3V3", bulk=True)
    C("C9", "4.7u", (295, 40), "VCAP1", bulk=True)
    note("one 100n per VDD pin, close to the pin;\nC9 on VCAP_1: low-ESR ceramic", 240, 52, 1.27)

    part("FB1", "Device", "FerriteBead_Small", "600R@100MHz", (245, 70),
         {"1": "+3V3", "2": "+3V3A"}, "Inductor_SMD:L_0603_1608Metric")
    part("#FLG03", "power", "PWR_FLAG", "PWR_FLAG", (265, 60), {"1": "+3V3A"})
    C("C10", "100n", (265, 70), "+3V3A")
    C("C11", "1u", (277, 70), "+3V3A")

    note("BOOT / RESET", 240, 90)
    R("R3", "10k", (245, 105), "BOOT0", "GND")
    C("C12", "100n", (258, 105), "NRST")
    part("SW1", "Switch", "SW_Push", "RESET", (275, 105), {"1": "NRST", "2": "GND"},
         "Button_Switch_SMD:SW_SPST_PTS810")

    note("STATUS LED + CALIBRATE BUTTON", 240, 125)
    R("R4", "1k", (245, 140), "LED_STATUS", "LED_A")
    part("D1", "Device", "LED", "green", (262, 140), {"A": "LED_A", "K": "GND"},
         "LED_SMD:LED_0603_1608Metric")
    R("R5", "4.7k", (245, 160), "+3V3", "BTN_CAL")
    C("C13", "100n", (258, 160), "BTN_CAL")
    part("SW2", "Switch", "SW_Push", "CAL", (275, 160), {"1": "BTN_CAL", "2": "GND"},
         "Button_Switch_SMD:SW_SPST_PTS810")

    # ---- IMU
    note("IMU  ICM-42688-P (SPI3, mode 0 or 3, <= 24 MHz)", 20, 135)
    part("U3", PROJECT, "ICM-42688-P", "ICM-42688-P", (60, 170), {
        "12": "IMU_CS", "13": "SPI3_SCK", "14": "SPI3_MOSI", "1": "SPI3_MISO",
        "4": "IMU_INT",
        "9": "GND",                     # FSYNC unused: datasheet says tie to GND
        "8": "+3V3_SENS", "5": "+3V3_SENS", "6": "GND",
        "7": "GND",                     # RESV, must be grounded
        "2": "GND", "3": "GND", "10": "GND", "11": "GND",   # RESV, NC or GND
    })
    C("C14", "100n", (100, 150), "+3V3_SENS")
    C("C15", "2.2u", (110, 150), "+3V3_SENS")
    C("C16", "10n", (120, 150), "+3V3_SENS")
    note("C14/C15 at VDD (pin 8), C16 at VDDIO (pin 5)", 95, 162, 1.27)
    note("No CS pull-up needed: PA15 comes out\nof reset as JTDI with its internal\npull-up, so CS stays high while the\nMCU is in reset and the IMU never\nlatches into I2C mode", 95, 182, 1.27)

    # ---- Barometer
    note("BARO  MS5611-01BA03 (I2C1, addr 0x77)", 150, 170)
    part("U5", "Sensor_Pressure", "MS5611-01BA", "MS5611-01BA03", (175, 190), {
        "VDD": "+3V3_SENS", "GND": "GND",
        "PS": "+3V3_SENS",              # PS high = I2C
        "CSB": "GND",                   # CSB low -> address LSB 1 -> 0x77
        "SCLK": "I2C1_SCL", "SDI/SDA": "I2C1_SDA",
    }, nc_ok=True)                      # SDO unused in I2C mode
    C("C22", "100n", (200, 185), "+3V3_SENS")
    R("R7", "4.7k", (210, 185), "+3V3_SENS", "I2C1_SCL")
    R("R8", "4.7k", (220, 185), "+3V3_SENS", "I2C1_SDA")
    note("C22 right at VDD (pin 1). No tracks between\nthe pads; cover the port with open-cell foam", 195, 207, 1.27)

    # ---- GPS
    note("GPS  SAM-M10Q (USART6)", 315, 25)
    part("U6", "Regulator_Linear", "LP5907MFX-3.3", "LP5907MFX-3.3", (340, 45),
         {"IN": "+5V", "EN": "+5V", "GND": "GND", "OUT": "+3V3_GPS"}, nc_ok=True)
    C("C23", "1u", (322, 45), "+5V")
    C("C24", "1u", (358, 45), "+3V3_GPS")
    part("U7", "RF_GPS", "SAM-M8Q", "SAM-M10Q", (345, 95), {
        "VCC": "+3V3_GPS", "VCC_IO": "+3V3_GPS",   # u-blox: tie VCC and V_IO together
        "GND": "GND",
        "RXD": "UART6_TX", "TXD": "UART6_RX",
    }, nc_ok=True)   # V_BCKP, RESET_N, SAFEBOOT_N/TIMEPULSE, EXTINT, I2C: leave open
    C("C25", "4.7u", (375, 85), "+3V3_GPS", bulk=True)
    C("C26", "100n", (385, 85), "+3V3_GPS")
    note("SAM-M8Q symbol/footprint: SAM-M10Q is pin- and\n"
         "footprint-compatible (u-blox UBX-22004868).\n"
         "Layout: middle of a solid >= 40x40 mm GND pour\n"
         "(50x50 ideal), top side, nothing above it, away\n"
         "from the nRF24, MCU and motor wiring.\n"
         "C24+C25+C26 = 5.8 uF, under LP5907's 10 uF limit.\n"
         "No capacitor on RESET_N. V_BCKP open = no hot start.\n"
         "UART default 9600 8N1 (UBX + NMEA).", 315, 120, 1.27)

    # ---- ESD on off-board signals (place each array at its connector)
    note("ESD  (at the connectors)", 315, 150)
    part("U8", "Power_Protection", "SP0504BAHT", "SP0504BAHT", (330, 175),
         {"1": "M1", "3": "M2", "4": "M3", "5": "M4", "2": "GND"})
    part("U9", "Power_Protection", "SP0504BAHT", "SP0504BAHT", (355, 175),
         {"1": "UART1_TX", "3": "UART1_RX", "4": "I2C3_SCL", "5": "I2C3_SDA", "2": "GND"})
    part("U10", "Power_Protection", "SP0504BAHT", "SP0504BAHT", (380, 175),
         {"1": "TELEM_TX", "3": "TELEM_RX", "2": "GND"}, nc_ok=True)
    note("U8 ESC (J4), U9 AUX (J3), U10 telemetry (J6)", 315, 192, 1.27)

    # ---- Radio
    note("RADIO  nRF24L01+ module (SPI2)", 20, 215)
    part("J2", "RF", "NRF24L01_Breakout", "nRF24L01+", (55, 245), {
        "VCC": "+3V3", "GND": "GND", "CE": "NRF_CE", "~{CSN}": "NRF_CSN",
        "SCK": "SPI2_SCK", "MOSI": "SPI2_MOSI", "MISO": "SPI2_MISO", "IRQ": "NRF_IRQ",
    })
    C("C18", "10u", (95, 240), "+3V3", bulk=True)
    C("C19", "100n", (105, 240), "+3V3")
    note("C18/C19 at the module pins:\nPA+LNA modules draw ~115 mA bursts", 85, 255, 1.27)

    note("AUX (USART1 + I2C3)", 130, 215)
    header("J3", "AUX", (145, 245), ["+5V", "UART1_TX", "UART1_RX", "I2C3_SCL", "I2C3_SDA", "GND"],
           jst_gh=True)
    R("R9", "4.7k", (165, 240), "+3V3", "I2C3_SCL")
    R("R10", "4.7k", (175, 240), "+3V3", "I2C3_SDA")
    note("JST-GH, Pixhawk GPS pinout:\n1 5V 2 TX 3 RX 4 SCL 5 SDA 6 GND\n"
         "I2C3 is separate from the baro bus,\nso 5 V pull-ups on an external\n"
         "module only reach 5V-tolerant pins", 130, 266, 1.27)

    # ---- Connectors
    note("CONNECTORS", 200, 215)
    header("J4", "ESC", (210, 245), ["M1", "M2", "M3", "M4", "GND"])
    header("J5", "SWD", (245, 245), ["+3V3", "SWCLK", "GND", "SWDIO", "NRST", "SWO"], jst_gh=True)
    note("JST-GH, pinout = Nucleo CN4:\nGH-to-Dupont cable to its ST-LINK", 235, 262, 1.27)
    header("J6", "TELEMETRY", (280, 245), ["GND", "TELEM_TX", "TELEM_RX"], jst_gh=True)


if __name__ == "__main__":
    write_project_lib([icm42688_symbol()], "FalconDrone parts")
    build()
    write_sch("FalconDrone flight controller")
