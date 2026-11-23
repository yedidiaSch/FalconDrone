#!/usr/bin/env python3
"""
Generate nrf24_adapter.kicad_sch: a 2x4 -> 1x8 adapter for an nRF24L01+
module, with the supply capacitors on board.

The module plugs into a 2x4 female socket; the 1x8 straight header underneath
fits a breadboard or any single-row socket. Pin order of the 1x8 follows the
module's own numbering: GND VCC CE CSN SCK MOSI MISO IRQ. 3.3 V in only.
JLCPCB places the two 0805 capacitors; the headers are hand-soldered.

Usage:  python3 gen_schematic.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import schgen  # noqa: E402
from schgen import C, header, init, note, part, write_sch  # noqa: E402

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT = "nrf24_adapter"
init(PROJECT, OUT_DIR)
schgen.C_FP = "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder"

PINS = ["GND", "+3V3", "CE", "CSN", "SCK", "MOSI", "MISO", "IRQ"]


def build():
    note("nRF24L01+ 2x4 -> 1x8 adapter", 20, 25)
    part("J1", "RF", "NRF24L01_Breakout", "nRF24L01+ socket", (45, 60), {
        "GND": "GND", "VCC": "+3V3", "CE": "CE", "~{CSN}": "CSN",
        "SCK": "SCK", "MOSI": "MOSI", "MISO": "MISO", "IRQ": "IRQ"})
    header("J2", "1x8 OUT", (95, 60), PINS)
    C("C1", "10u", (120, 55), "+3V3")
    C("C2", "100n", (130, 55), "+3V3")
    note("C1/C2 at the module's VCC pin: PA+LNA modules draw ~115 mA bursts", 105, 70, 1.27)
    part("#FLG01", "power", "PWR_FLAG", "PWR_FLAG", (120, 40), {"1": "+3V3"})
    part("#FLG02", "power", "PWR_FLAG", "PWR_FLAG", (130, 40), {"1": "GND"})


if __name__ == "__main__":
    build()
    write_sch("nRF24 2x4 to 1x8 adapter")
