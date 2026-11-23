#!/usr/bin/env python3
"""
Generate falcon_carrier.kicad_sch: a carrier board for the bench Nucleo-F446RE.

The Nucleo plugs in from above: its ST morpho pins (CN7, CN10) come out of
both sides of the board and go into two 2x19 female sockets on the carrier.
The carrier replaces the jumper wires: it carries the IMU, the nRF24 socket,
the 5 V input from the ESC BEC, battery sensing, the ESC header and a
buzzer. JLCPCB assembles the SMD parts except the IMU (gen_fab.py); the
IMU and every through-hole part are hand-soldered.

Pin assignment follows falconDrone.ioc, so the bench firmware's pin map
carries over unchanged:
  ESC M1..M4   PA0 PA1 (TIM2 CH1/CH2)  PA6 PA7 (TIM3 CH1/CH2)
  nRF24L01     SPI2  PB13 SCK / PB14 MISO / PB15 MOSI, CSN PB12, CE PB1, IRQ PC4
  IMU          I2C1  PB6 SCL / PB7 SDA (the bus the MPU-6050 used), address 0x69
New pins, not yet in the .ioc:  PA4 VBAT_SENSE (ADC1_IN4), PB0 BUZZER.

ICM-20948 (TDK DS-000189): VDDIO is 1.71-1.95 V, so the chip cannot sit on the
3.3 V bus directly. It runs from its own 1.8 V LDO (VDD and VDDIO both, which
also keeps it off the rail the nRF24 pulls on) and a PCA9306 translates I2C.
INT1 is left open: the firmware polls.

Power: 5 V from one ESC's BEC (J7) -> Nucleo E5V. Move the Nucleo's JP5
jumper to E5V to run from it (USB then still works for ST-LINK and telemetry,
but the target is only powered while the battery is in). The Nucleo's own
3.3 V regulator powers the IMU LDO and the nRF24. The battery pads (J3) only
feed the voltage divider. Only one BEC may be connected: the ESC header (J6)
carries signals and GND, no 5 V.

Usage:  python3 gen_schematic.py   (then: kicad-cli sch erc falcon_carrier.kicad_sch)
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import schgen  # noqa: E402
from schgen import C, R, header, init, note, part, write_sch  # noqa: E402

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT = "falcon_carrier"
init(PROJECT, OUT_DIR)

# Through-hole resistors, hand-soldered
R_THT = "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal"
# SMD, placed by JLCPCB (hand-solder pads: easy to rework)
R_0805 = "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder"
C_0805 = "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder"
schgen.R_FP, schgen.C_FP = R_0805, C_0805      # what R() / C() use

# Nucleo morpho pins (UM1724 table 29, NUCLEO-F446RE). The footprint is a
# plain 2x19 pin header: seen from the top it has the same pin positions as
# the Nucleo seen from the top, which is what a female socket under the
# Nucleo needs (a KiCad PinSocket footprint would be mirrored).
CN7 = {
    "6": "+5V",            # E5V: external 5 V input, JP5 on E5V
    "8": "GND", "19": "GND", "20": "GND", "22": "GND",
    "16": "+3V3",          # Nucleo LDO output
    "21": "I2C1_SDA",      # PB7
    "28": "M1",            # PA0
    "30": "M2",            # PA1
    "32": "VBAT_SENSE",    # PA4
    "34": "BUZZER",        # PB0
}
CN10 = {
    "9": "GND", "20": "GND",
    "17": "I2C1_SCL",      # PB6
    "13": "M3",            # PA6
    "15": "M4",            # PA7
    "16": "NRF_CSN",       # PB12
    "24": "NRF_CE",        # PB1
    "26": "SPI2_MOSI",     # PB15
    "28": "SPI2_MISO",     # PB14
    "30": "SPI2_SCK",      # PB13
    "34": "NRF_IRQ",       # PC4
}
MORPHO_FP = "Connector_PinHeader_2.54mm:PinHeader_2x19_P2.54mm_Vertical"


def build():
    # ---- Nucleo
    note("NUCLEO-F446RE  (plugs in from above)", 20, 25)
    part("J1", "Connector_Generic", "Conn_02x19_Odd_Even", "CN7 socket", (40, 70), CN7,
         MORPHO_FP, nc_ok=True)
    part("J2", "Connector_Generic", "Conn_02x19_Odd_Even", "CN10 socket", (90, 70), CN10,
         MORPHO_FP, nc_ok=True)
    note("2x19 female sockets, 8.5 mm. Footprint = Nucleo top view.\n"
         "Nucleo JP5 on E5V to run from the battery.", 20, 105, 1.27)

    # ---- Power
    note("POWER  (5 V from the ESC BEC)", 130, 25)
    header("J7", "BEC 5V", (135, 50), ["+5V", "GND"])
    C("C2", "22u", (155, 55), "+5V")
    note("one ESC's BEC: 1 +5 V (red), 2 GND (black)", 130, 62, 1.27)
    part("J3", "Connector_Generic", "Conn_01x02", "BATTERY", (185, 50),
         {"1": "VBAT", "2": "GND"}, "Connector_Wire:SolderWire-0.5sqmm_1x02_P4.8mm_D0.9mm_OD2.3mm")
    note("solder pads, sense only: 1 battery +, 2 GND", 180, 62, 1.27)
    part("#FLG01", "power", "PWR_FLAG", "PWR_FLAG", (150, 35), {"1": "+5V"})
    part("#FLG02", "power", "PWR_FLAG", "PWR_FLAG", (165, 35), {"1": "VBAT"})
    part("#FLG03", "power", "PWR_FLAG", "PWR_FLAG", (195, 35), {"1": "GND"})
    part("#FLG04", "power", "PWR_FLAG", "PWR_FLAG", (55, 35), {"1": "+3V3"})

    note("BATTERY SENSE  (6S 25.2 V -> 2.26 V on PA4)", 130, 85)
    part("R1", "Device", "R", "10k", (140, 100), {"1": "VBAT", "2": "VBAT_SENSE"}, R_THT)
    part("R2", "Device", "R", "1k", (155, 100), {"1": "VBAT_SENSE", "2": "GND"}, R_THT)
    C("C3", "100n", (170, 100), "VBAT_SENSE")
    note("battery reversed: R1 limits PA4 injection to 2.5 mA", 130, 112, 1.27)

    # ---- IMU
    note("IMU  ICM-20948  (I2C1 through PCA9306, addr 0x69)", 20, 125)
    part("U2", "Regulator_Linear", "MCP1700x-180xxTT", "MCP1700T-1802E/TT", (30, 150),
         {"VI": "+3V3", "VO": "+1V8", "GND": "GND"})
    C("C4", "1u", (20, 165), "+3V3")
    C("C5", "1u", (42, 165), "+1V8")
    part("U3", "Sensor_Motion", "ICM-20948", "ICM-20948", (80, 165), {
        "13": "+1V8",          # VDD
        "8": "+1V8",           # VDDIO, 1.71-1.95 V
        "18": "GND", "20": "GND",
        "10": "IMU_REGOUT",
        "9": "+1V8",           # AD0 high -> 0x69; tied to its neighbour pin 8, a GND via there boxes in REGOUT
        "22": "+1V8",          # CS high -> I2C mode
        "11": "GND",           # FSYNC unused: tie low
        "23": "IMU_SCL", "24": "IMU_SDA",
    }, nc_ok=True)             # INT1, AUX_CL/AUX_DA, NC pins
    C("C6", "100n", (105, 150), "+1V8")
    C("C7", "100n", (115, 150), "+1V8")
    C("C8", "100n", (125, 150), "IMU_REGOUT")
    note("C6 at VDD (13), C7 at VDDIO (8), C8 at REGOUT (10)", 100, 162, 1.27)

    part("U4", "Interface", "PCA9306DC", "PCA9306DCUR", (160, 165), {
        "GND": "GND", "VREF1": "+1V8", "VREF2": "VREF2", "EN": "VREF2",
        "SCL1": "IMU_SCL", "SDA1": "IMU_SDA", "SCL2": "I2C1_SCL", "SDA2": "I2C1_SDA",
    })
    R("R3", "200k", (185, 150), "+3V3", "VREF2")
    C("C9", "100n", (195, 150), "VREF2")
    part("#FLG05", "power", "PWR_FLAG", "PWR_FLAG", (195, 135), {"1": "VREF2"})   # fed through R3
    R("R4", "2.2k", (135, 185), "+1V8", "IMU_SCL")
    R("R5", "2.2k", (145, 185), "+1V8", "IMU_SDA")
    R("R6", "4.7k", (175, 185), "+3V3", "I2C1_SCL")
    R("R7", "4.7k", (185, 185), "+3V3", "I2C1_SDA")
    note("PCA9306: VREF1 = 1.8 V side, VREF2 = EN via 200k to 3.3 V\n"
         "(TI SCPS113). Pull-ups on both sides; the Nucleo has none.", 130, 200, 1.27)
    header("J4", "I2C TEST", (215, 170), ["+3V3", "I2C1_SCL", "I2C1_SDA", "GND"])
    note("test the IMU here before\nthe Nucleo goes on", 205, 185, 1.27)

    # ---- Radio
    note("RADIO  nRF24L01+ module (SPI2)", 20, 215)
    part("J5", "RF", "NRF24L01_Breakout", "nRF24L01+", (45, 245), {
        "VCC": "+3V3", "GND": "GND", "CE": "NRF_CE", "~{CSN}": "NRF_CSN",
        "SCK": "SPI2_SCK", "MOSI": "SPI2_MOSI", "MISO": "SPI2_MISO", "IRQ": "NRF_IRQ",
    })
    C("C10", "10u", (80, 240), "+3V3")
    C("C11", "100n", (90, 240), "+3V3")
    note("2x4 female socket; C10/C11 at the module\n(PA+LNA modules draw ~115 mA bursts)", 70, 255, 1.27)

    # ---- ESC + buzzer
    note("ESC", 120, 215)
    header("J6", "ESC", (125, 245), ["M1", "M2", "M3", "M4", "GND"])
    note("signal wires only: leave the\nESC BEC (+5 V) wire unconnected", 115, 262, 1.27)

    note("BUZZER  (active, 5 V)", 165, 215)
    part("BZ1", "Device", "Buzzer", "5V active", (200, 235), {"+": "+5V", "-": "BUZ_N"},
         "Buzzer_Beeper:Buzzer_12x9.5RM7.6")
    part("Q1", "Transistor_BJT", "2N3904", "2N3904", (195, 255),
         {"C": "BUZ_N", "B": "BUZ_B", "E": "GND"})
    part("R8", "Device", "R", "1k", (175, 255), {"1": "BUZZER", "2": "BUZ_B"}, R_THT)
    part("R9", "Device", "R", "10k", (182, 270), {"1": "BUZ_B", "2": "GND"}, R_THT)
    note("R9 keeps it quiet while the MCU is in reset", 165, 282, 1.27)

    # ---- Mounting
    for i in range(7):
        part(f"H{i + 1}", "Mechanical", "MountingHole", "M3", (240 + 12 * (i % 4), 40 + 15 * (i // 4)),
             {}, "MountingHole:MountingHole_3.2mm_M3", in_bom=False)
    note("H1-H4 frame (corners), H5-H7 Nucleo standoffs", 235, 75, 1.27)


if __name__ == "__main__":
    build()
    write_sch("FalconDrone Nucleo carrier")
