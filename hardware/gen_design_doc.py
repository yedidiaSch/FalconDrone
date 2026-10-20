#!/usr/bin/env python3
"""
Build FalconDrone_hardware_design.pdf: the hardware design description of the
handheld controller (falcon_rc) and the flight controller (falcon_fc).

Board renders, ERC/DRC results and the controller BOM are taken from the
current KiCad files each run, so the document follows the design; the prose
lives below. Both schematics are appended as the last pages.

Needs kicad-cli, pdfunite (poppler-utils) and reportlab.
Usage:  python3 gen_design_doc.py
"""

import os
import re
import subprocess
import sys
from datetime import date

import pcbnew
from PIL import Image
from reportlab.graphics.shapes import Drawing, Line, Polygon, Rect, String
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (Image as RLImage, KeepTogether, PageBreak, Paragraph,
                                SimpleDocTemplate, Spacer, Table, TableStyle)

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, "build", "doc")
OUT = os.path.join(HERE, "FalconDrone_hardware_design.pdf")
RC = os.path.join(HERE, "controller", "falcon_rc")
FC = os.path.join(HERE, "flight_controller", "falcon_fc")

sys.path.insert(0, os.path.join(HERE, "controller"))
import gen_fab  # noqa: E402  (controller part numbers)

# ---------------------------------------------------------------- styles

FONT_DIR = "/usr/share/fonts/truetype/dejavu"
pdfmetrics.registerFont(TTFont("DejaVu", f"{FONT_DIR}/DejaVuSans.ttf"))
pdfmetrics.registerFont(TTFont("DejaVu-Bold", f"{FONT_DIR}/DejaVuSans-Bold.ttf"))
pdfmetrics.registerFont(TTFont("DejaVuMono", f"{FONT_DIR}/DejaVuSansMono.ttf"))
pdfmetrics.registerFontFamily("DejaVu", normal="DejaVu", bold="DejaVu-Bold")

INK = colors.HexColor("#1f2933")
ACCENT = colors.HexColor("#1d5c8f")
MUTED = colors.HexColor("#5f6b7a")
RULE = colors.HexColor("#c9d2dc")
SHADE = colors.HexColor("#eef2f6")

ss = getSampleStyleSheet()
BODY = ParagraphStyle("body", ss["BodyText"], fontName="DejaVu", fontSize=9.2, leading=13,
                      textColor=INK, spaceAfter=5)
SMALL = ParagraphStyle("small", BODY, fontSize=7.8, leading=10, textColor=MUTED)
CELL = ParagraphStyle("cell", BODY, fontSize=7.8, leading=9.6, spaceAfter=0)
CELLB = ParagraphStyle("cellb", CELL, fontName="DejaVu-Bold")
H1 = ParagraphStyle("h1", BODY, fontName="DejaVu-Bold", fontSize=16, leading=20,
                    textColor=ACCENT, spaceBefore=4, spaceAfter=8)
H2 = ParagraphStyle("h2", BODY, fontName="DejaVu-Bold", fontSize=11.5, leading=15,
                    textColor=INK, spaceBefore=10, spaceAfter=4)
H3 = ParagraphStyle("h3", BODY, fontName="DejaVu-Bold", fontSize=9.6, leading=13,
                    textColor=ACCENT, spaceBefore=6, spaceAfter=2)
TITLE = ParagraphStyle("title", BODY, fontName="DejaVu-Bold", fontSize=26, leading=32,
                       textColor=INK, alignment=TA_CENTER)
SUB = ParagraphStyle("sub", BODY, fontSize=12, leading=16, textColor=MUTED, alignment=TA_CENTER)
BULLET = ParagraphStyle("bullet", BODY, leftIndent=12, bulletIndent=2, spaceAfter=2)


def p(text, style=BODY):
    return Paragraph(text, style)


def bullets(items):
    return [Paragraph(t, BULLET, bulletText="•") for t in items]


def table(rows, widths, header=True, mono_cols=()):
    data = []
    for r, row in enumerate(rows):
        out = []
        for c, v in enumerate(row):
            st = CELLB if header and r == 0 else CELL
            if c in mono_cols and not (header and r == 0):
                st = ParagraphStyle("m", CELL, fontName="DejaVuMono", fontSize=7.4)
            out.append(Paragraph(re.sub(r"&(?!#?\w+;)", "&amp;", str(v)), st))
        data.append(out)
    t = Table(data, colWidths=[w * mm for w in widths], repeatRows=1 if header else 0)
    style = [("VALIGN", (0, 0), (-1, -1), "TOP"),
             ("LINEBELOW", (0, 0), (-1, -1), 0.4, RULE),
             ("TOPPADDING", (0, 0), (-1, -1), 2.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5),
             ("LEFTPADDING", (0, 0), (-1, -1), 3), ("RIGHTPADDING", (0, 0), (-1, -1), 3)]
    if header:
        style += [("BACKGROUND", (0, 0), (-1, 0), SHADE), ("LINEBELOW", (0, 0), (-1, 0), 0.8, ACCENT)]
    t.setStyle(TableStyle(style))
    return t


def image(path, width_mm):
    w, h = Image.open(path).size
    return RLImage(path, width=width_mm * mm, height=width_mm * mm * h / w)


def caption(text):
    return p(text, ParagraphStyle("cap", SMALL, alignment=TA_CENTER, spaceBefore=2, spaceAfter=8))


# ---------------------------------------------------------------- data from the design

def run(cmd):
    return subprocess.run(cmd, check=True, capture_output=True, text=True).stdout


def checks(base):
    """ERC / DRC (with schematic parity) counts, as kicad-cli reports them."""
    erc_rpt = os.path.join(BUILD, os.path.basename(base) + ".erc.rpt")
    drc_rpt = os.path.join(BUILD, os.path.basename(base) + ".drc.rpt")
    erc = run(["kicad-cli", "sch", "erc", "-o", erc_rpt, base + ".kicad_sch"])
    drc = run(["kicad-cli", "pcb", "drc", "--schematic-parity", "-o", drc_rpt, base + ".kicad_pcb"])
    num = lambda pat, s: int(re.search(pat, s).group(1))
    kinds = {}
    for line in open(drc_rpt):
        m = re.match(r"\[(\w+)\]", line)
        if m:
            kinds[m.group(1)] = kinds.get(m.group(1), 0) + 1
    return {"erc": num(r"Found (\d+) violations", erc),
            "unconnected": num(r"Found (\d+) unconnected", drc),
            "parity": num(r"Found (\d+) schematic parity", drc),
            "drc": kinds}


def render(base, side, zoom, size):
    """Board render, cropped to the board."""
    png = os.path.join(BUILD, f"{os.path.basename(base)}_{side}.png")
    run(["kicad-cli", "pcb", "render", "--side", side, "--width", str(size[0]),
         "--height", str(size[1]), "--quality", "basic", "--background", "opaque",
         "--zoom", str(zoom), "-o", png, base + ".kicad_pcb"])
    im = Image.open(png).convert("RGB")
    px, (w, h) = im.load(), im.size
    pts = [(x, y) for y in range(0, h, 3) for x in range(0, w, 3)
           if px[x, y][2] < 120 and px[x, y][1] > px[x, y][0] - 10]      # board green
    xs, ys = [q[0] for q in pts], [q[1] for q in pts]
    jpg = png[:-4] + ".jpg"
    im.crop((max(min(xs) - 8, 0), max(min(ys) - 8, 0), min(max(xs) + 8, w), min(max(ys) + 8, h))
            ).save(jpg, quality=85)
    return jpg


def board_stats(base):
    b = pcbnew.LoadBoard(base + ".kicad_pcb")
    tracks = [t for t in b.GetTracks() if t.GetClass() == "PCB_TRACK"]
    vias = [t for t in b.GetTracks() if t.GetClass() == "PCB_VIA"]
    bb = b.GetBoardEdgesBoundingBox()
    return {"fps": len(b.GetFootprints()), "layers": b.GetCopperLayerCount(),
            "tracks": len(tracks), "vias": len(vias),
            "size": (round(pcbnew.ToMM(bb.GetWidth())), round(pcbnew.ToMM(bb.GetHeight()))),
            "board": b}


def controller_bom(board):
    fps = [fp for fp in board.GetFootprints() if gen_fab.assembled(fp)]
    groups = {}
    for fp in fps:
        lcsc, mpn = gen_fab.part_for(fp)
        groups.setdefault((lcsc, mpn, "SMD" if fp.GetAttributes() & pcbnew.FP_SMD else "THT"),
                          []).append(fp.GetReference())
    rows = [["Qty", "Designators", "Part", "LCSC", "Side"]]
    for (lcsc, mpn, kind), refs in sorted(groups.items(), key=lambda kv: gen_fab.natural(
            sorted(kv[1], key=gen_fab.natural)[0])):
        refs.sort(key=gen_fab.natural)
        rows.append([len(refs), ", ".join(refs), mpn, lcsc or "Global Sourcing",
                     "back (reflow)" if kind == "SMD" else "front (THT)"])
    return rows, len(fps)


# ---------------------------------------------------------------- block diagrams

def box(d, x, y, w, h, title, sub="", fill=SHADE, stroke=ACCENT):
    d.add(Rect(x, y, w, h, rx=3, ry=3, fillColor=fill, strokeColor=stroke, strokeWidth=0.8))
    d.add(String(x + w / 2, y + h / 2 + (2.5 if sub else -3), title, fontName="DejaVu-Bold",
                 fontSize=7.4, fillColor=INK, textAnchor="middle"))
    if sub:
        d.add(String(x + w / 2, y + h / 2 - 7.5, sub, fontName="DejaVu", fontSize=6.2,
                     fillColor=MUTED, textAnchor="middle"))


def arrow(d, x0, y0, x1, y1, label="", color=INK, dy=3):
    d.add(Line(x0, y0, x1, y1, strokeColor=color, strokeWidth=0.8))
    import math
    a = math.atan2(y1 - y0, x1 - x0)
    s = 4
    d.add(Polygon([x1, y1, x1 - s * math.cos(a - 0.4), y1 - s * math.sin(a - 0.4),
                   x1 - s * math.cos(a + 0.4), y1 - s * math.sin(a + 0.4)],
                  fillColor=color, strokeColor=color))
    if label and x0 == x1:                    # vertical: label beside the line
        d.add(String(x0 + 4, (y0 + y1) / 2 - 2, label, fontName="DejaVu", fontSize=5.8,
                     fillColor=MUTED, textAnchor="start"))
    elif label:
        d.add(String((x0 + x1) / 2, (y0 + y1) / 2 + dy, label, fontName="DejaVu", fontSize=5.8,
                     fillColor=MUTED, textAnchor="middle"))


def system_diagram():
    d = Drawing(170 * mm, 52 * mm)
    W = 170 * mm
    box(d, 2, 70, 150, 70, "Handheld controller", "falcon_rc · ESP32-S3 · 18650")
    box(d, W - 152, 70, 150, 70, "Flight controller", "falcon_fc · STM32F446 · IMU/baro/GPS")
    arrow(d, 152, 112, W - 152, 112, "nRF24L01+ 2.4 GHz  (controller PTX → drone PRX)", ACCENT)
    arrow(d, W - 152, 96, 152, 96, "ACK payload telemetry", ACCENT, dy=-8)
    box(d, 2, 4, 150, 44, "USB-C", "charge · flash · console", fill=colors.white)
    arrow(d, 77, 48, 77, 70)
    box(d, W - 152, 4, 70, 44, "4 × ESC", "PWM M1–M4", fill=colors.white)
    box(d, W - 76, 4, 74, 44, "Battery 2–6S", "+5 V from BEC", fill=colors.white)
    arrow(d, W - 117, 70, W - 117, 48)
    arrow(d, W - 39, 48, W - 39, 70)
    return d


def controller_diagram():
    d = Drawing(170 * mm, 92 * mm)
    # power column
    box(d, 2, 210, 78, 40, "USB-C", "VBUS 5 V, ESD")
    box(d, 2, 150, 78, 44, "BQ24073", "power path charger")
    box(d, 2, 82, 78, 50, "18650 cell", "DW01A + 8205A prot.")
    box(d, 110, 150, 82, 44, "AP2112K-3.3", "LDO, EN = power sw")
    box(d, 110, 82, 82, 44, "Ferrite + caps", "+3V3A for sticks")
    arrow(d, 41, 210, 41, 194, "VBUS")
    arrow(d, 41, 150, 41, 132, "BAT (charge / supply)")
    arrow(d, 80, 172, 110, 172, "VSYS")
    arrow(d, 151, 150, 151, 126, "+3V3", dy=0)
    # MCU
    box(d, 230, 96, 120, 134, "ESP32-S3-WROOM-1", "N8 flash · native USB · ADC1", fill=colors.white)
    arrow(d, 192, 172, 230, 172, "+3V3")
    arrow(d, 80, 230, 230, 215, "USB D+/D-", dy=4)
    # peripherals right
    px = 385
    for i, (t, s, lab) in enumerate((("nRF24L01+ PA/LNA", "socket, SPI", "FSPI"),
                                     ("SSD1306 OLED", "socket, I2C 0x3C", "I2C"),
                                     ("Buzzer + LEDs", "piezo, status", "PWM"))):
        y = 200 - i * 48
        box(d, px, y, 92, 36, t, s)
        arrow(d, 350, y + 18, px, y + 18, lab)
    # inputs
    box(d, 230, 10, 120, 56, "Controls", "2 sticks · 4 buttons · 2 toggles", fill=colors.white)
    arrow(d, 290, 66, 290, 96, "ADC1 ×4 + GPIO")
    arrow(d, 192, 104, 230, 104, "VPOT / VBAT", dy=-8)
    return d


# ---------------------------------------------------------------- the document

def build():
    os.makedirs(BUILD, exist_ok=True)
    rc_chk, fc_chk = checks(RC), checks(FC)
    rc, fc = board_stats(RC), board_stats(FC)
    rc_top, rc_bot = render(RC, "top", 1.3, (1600, 1100)), render(RC, "bottom", 1.3, (1600, 1100))
    fc_top, fc_bot = render(FC, "top", 0.9, (1200, 1200)), render(FC, "bottom", 0.9, (1200, 1200))
    bom_rows, n_assembled = controller_bom(rc["board"])
    today = date.today().isoformat()

    s = []
    # ---- cover
    s += [Spacer(1, 45 * mm), p("FalconDrone", TITLE), Spacer(1, 3 * mm),
          p("Hardware design description", SUB), Spacer(1, 2 * mm),
          p("Handheld controller (falcon_rc) and flight controller (falcon_fc), rev A", SUB),
          Spacer(1, 14 * mm), image(rc_top, 150), caption("Controller, front side"),
          Spacer(1, 10 * mm),
          p(f"Generated {today} by hardware/gen_design_doc.py from the KiCad 9 projects. "
            "The schematics, footprints and boards are themselves generated by the scripts "
            "described in section 4: edit the scripts, not the KiCad files.",
            ParagraphStyle("c", SMALL, alignment=TA_CENTER)),
          PageBreak()]

    # ---- 1 system
    s += [p("1  System overview", H1),
          p("FalconDrone is a quadcopter built around a custom STM32F446 flight controller. "
            "This document covers the two custom boards: the <b>flight controller</b> that "
            "flies the drone, and the <b>handheld controller</b> the pilot holds. They talk "
            "over an nRF24L01+ 2.4 GHz link: the controller sends stick and switch frames, the "
            "drone answers with telemetry in the ACK payload."),
          system_diagram(), caption("Figure 1 – system blocks"),
          table([["", "Handheld controller", "Flight controller"],
                 ["Project", "hardware/controller/falcon_rc", "hardware/flight_controller/falcon_fc"],
                 ["MCU", "ESP32-S3-WROOM-1-N8 (Xtensa LX7, native USB)", "STM32F446RET6 (Cortex-M4F, 180 MHz)"],
                 ["Board", f"{rc['size'][0]} × {rc['size'][1]} mm, {rc['layers']} layers, 1.6 mm",
                  f"{fc['size'][0]} × {fc['size'][1]} mm, {fc['layers']} layers, 1.6 mm, 30.5 mm mount"],
                 ["Power", "1 × 18650 Li-ion, USB-C charging", "2–6S flight pack sense, 5 V from ESC BEC"],
                 ["Radio", "nRF24L01+ PA/LNA module (PTX)", "nRF24L01+ module (PRX)"],
                 ["Parts", f"{rc['fps']} footprints, {n_assembled} assembled", f"{fc['fps']} footprints"],
                 ["Status", "routed, DRC clean, JLCPCB full-assembly files", "routed, DRC clean"]],
                [28, 71, 71]),
          PageBreak()]

    # ---- 2 controller
    s += [p("2  Handheld controller (falcon_rc)", H1),
          p("A gamepad-shaped 150 × 100 mm board, held in both hands. Mode-2 layout: left "
            "stick throttle / yaw, right stick pitch / roll, a four-button menu cluster "
            "between the sticks, ARM and MODE toggles at the top corners, OLED and status "
            "LEDs top centre. The 18650 cell and every SMD part sit on the back, so the front "
            "carries only the controls and the board needs one reflow pass."),
          controller_diagram(), caption("Figure 2 – controller blocks"),
          p("2.1  Power path", H2),
          p("USB-C feeds a TI <b>BQ24073</b> linear charger with power-path management: its "
            "OUT pin (VSYS) is regulated to 4.4 V while USB is present and switches to the "
            "battery otherwise, so the controller runs while charging and charges while "
            "switched off. The cell is protected in its negative lead by a <b>DW01A</b> with "
            "a dual N-MOSFET <b>8205A</b>; board GND is pack minus. An <b>AP2112K-3.3</b> LDO "
            "makes +3V3. The power slide switch drives the LDO enable pin, so it carries no "
            "load current, and the charge LEDs stay on VSYS so charging is visible with the "
            "controller off."),
          table([["Setting", "Part", "Value", "Result (TI SLUS810N)"],
                 ["Fast-charge current", "R3 ISET", "1.2 kΩ", "890 AΩ / 1.2 kΩ = 0.74 A (0.66–0.81 A)"],
                 ["Input current limit", "R4 ILIM", "1.5 kΩ", "1610 AΩ / 1.5 kΩ = 1.07 A (EN2 = 1, EN1 = 0)"],
                 ["Safety timer", "R24 TMR", "68 kΩ", "10 × 48 s/kΩ × 68 kΩ = 9.1 h (the 5 h default "
                  "could stop a 3.5 Ah cell early)"],
                 ["Termination", "pin 15 TD", "GND", "termination on, at 10 % of the charge current"],
                 ["Battery temperature", "R5 TS", "10 kΩ", "fixed: NTC check off (no NTC in the holder)"],
                 ["Input OVP", "—", "6.6 V", "USB only; suspends above, survives 26 V"],
                 ["Thermal", "EP + 4 vias", "—", "≈ 1 W at 3.6 V cell; TJ(REG) 125 °C throttles beyond"]],
                [32, 22, 16, 100]),
          Spacer(1, 4),
          p("<b>Why the BQ24073 and not the BQ24074 of the first draft:</b> JLCPCB stocked one "
            "BQ24074. The '73 has the same 4.4 V OUT and the same ISET/ILIM/TMR constants; it "
            "differs in OVP (6.6 V instead of 10.5 V, irrelevant on USB) and in pin 15, "
            "TD instead of ITERM, tied low for the same 10 % termination.", SMALL),
          p("2.2  Protection, supply budget and sensing", H2),
          table([["Block", "Design", "Notes"],
                 ["Cell protection", "DW01A + 8205A (TSSOP-8), R8 100 Ω / C4 100 nF supply filter, "
                  "R9 1 kΩ CS", "over-charge 4.3 V, over-discharge ~2.4–2.5 V, short circuit via "
                  "the FETs' R<sub>DS(on)</sub>"],
                 ["3.3 V rail", "AP2112K-3.3, 600 mA, C5/C6 10 µF", "peak load ≈ 350 mA (ESP32-S3 RF "
                  "+ nRF24 PA bursts), ≈ 0.17 W typical in the LDO. Drops out below ≈ 3.5 V "
                  "cell: firmware warns at 3.6 V, shuts down at 3.4 V"],
                 ["Analog rail", "FB1 600 Ω ferrite, C8 10 µF + C9 100 nF → +3V3A", "feeds the "
                  "four stick pots; VPOT_SENSE reads it so stick values are ratiometric"],
                 ["Sense dividers", "VBAT 220k/220k, VPOT 100k/100k, VBUS 100k/150k", "1:2, 1:2 "
                  "and 0.6 into ADC1; 100 nF / 10 nF at the pins; µA drain when off"],
                 ["USB", "HRO TYPE-C-31-M-12, 5.1 kΩ CC pull-downs, USBLC6-2SC6", "native ESP32-S3 "
                  "USB: flashing, console and charging on one port"]],
                [28, 64, 78]),
          p("2.3  ESP32-S3 pin map", H2),
          p("ADC inputs use ADC1 only (ADC2 conflicts with Wi-Fi). IO35–37 stay free so an "
            "octal-PSRAM module variant also fits; the 1.8 V “V” variants are excluded "
            "(their IO47/48 run at 1.8 V). Buttons and switches pull to GND with the "
            "internal pull-ups."),
          table([["Function", "GPIO", "Function", "GPIO"],
                 ["STICK_LX (yaw)", "IO1", "OLED SDA / SCL", "IO8 / IO9"],
                 ["STICK_LY (throttle)", "IO2", "nRF24 CSN / MOSI / SCK / MISO", "IO10 / 11 / 12 / 13"],
                 ["STICK_RX (roll)", "IO4", "nRF24 CE / IRQ", "IO14 / IO21"],
                 ["STICK_RY (pitch)", "IO5", "BTN_L / BTN_R (stick press)", "IO15 / IO16"],
                 ["VBAT_SENSE", "IO6", "BTN UP / DOWN / OK / BACK", "IO17 / 18 / 38 / 39"],
                 ["VPOT_SENSE", "IO7", "SW_ARM", "IO40"],
                 ["VBUS_DET", "IO3", "SW_MODE_A / B (centre = neither)", "IO41 / IO42"],
                 ["USB D- / D+", "IO19 / IO20", "BUZZER / LED_STATUS", "IO47 / IO48"],
                 ["BOOT button", "IO0", "Debug UART (J2, not fitted)", "TXD0 / RXD0"]],
                [40, 25, 65, 40], mono_cols=(1, 3)),
          p("2.4  Radio, display and controls", H2),
          *bullets([
              "<b>Radio:</b> nRF24L01+ PA/LNA module in a 2 × 4 socket (J3) on FSPI IO-MUX pins, "
              "22 µF + 100 nF at the socket for the ≈ 115 mA TX bursts. Its SMA antenna points "
              "off the top edge, at the opposite end from the ESP32's PCB antenna, whose "
              "keep-out is clear of copper on both layers.",
              "<b>Display:</b> 0.96\" SSD1306 module in a 1 × 4 socket (J4), pin order GND VCC "
              "SCL SDA, 4.7 kΩ I2C pull-ups. Some modules swap GND/VCC: check before plugging in.",
              "<b>Sticks:</b> bare Alps RKJXV1224005 (two 10 kΩ pots + push switch) on a custom "
              "footprint drawn from Alps drawing KJXV14005. Frame tabs soldered to GND; "
              "C18–C21 100 nF at the ADC pins. The left stick's Y spring can be removed for a "
              "non-centring throttle.",
              "<b>Switches:</b> C&amp;K 7101 (ARM, ON-ON) and 7103 (MODE, ON-OFF-ON: ANGLE / RATE / "
              "spare) with PC pins on 4.70 mm, 1.85 mm holes per C&amp;K's layout. MTS-102/103 "
              "clones fit the same footprint. Four 6 × 6 mm tact switches form the menu cluster.",
              "<b>Buzzer:</b> CUI CPT-9019S piezo (12 nF, 4 kHz) switched by an MMBT3904; "
              "R25 1 kΩ across it discharges the piezo while the transistor is off.",
              "<b>LEDs:</b> 3 mm through-hole: green PWR (PGOOD), red CHG (charging), yellow status "
              "(IO48). Yellow because a blue LED's ≈ 3.4 V forward voltage would not light from "
              "a 3.3 V pin.",
              "<b>Reset / boot:</b> SMD RESET and BOOT buttons on the back, R17/C15 delay EN "
              "until 3V3 is up. Native USB normally auto-resets into download mode."]),
          PageBreak()]

    rc_drc = ", ".join(f"{v} {k.replace('_', ' ')}" for k, v in sorted(rc_chk["drc"].items())
                       if k != "unconnected_items") or "none"
    s += [p("2.5  PCB", H2),
          table([["Item", "Value"],
                 ["Outline", "150 × 100 mm, 6 mm corner radius, 4 × M3 holes 5 mm in from the corners"],
                 ["Stack-up", "2 layers, 1.6 mm FR-4, 1 oz; GND pours on both layers"],
                 ["Rules", "JLCPCB standard: 0.15 mm clearance, 0.3 mm min drill, 0.6 mm via"],
                 ["Placement", "front: sticks, toggles, tact switches, LEDs, nRF24 and OLED sockets; "
                  "back: every SMD part, 18650 holder, USB-C, power switch"],
                 ["Routing", f"{rc['tracks']} track segments, {rc['vias']} vias (incl. GND fan-out, "
                  "charger thermal vias and the stitching grid)"],
                 ["Checks", f"ERC {rc_chk['erc']} violations · {rc_chk['unconnected']} unconnected · "
                  f"{rc_chk['parity']} schematic-parity issues · DRC remaining: {rc_drc}"]],
                [28, 142]),
          Spacer(1, 3),
          p("Remaining DRC entries are expected: the ESP32 footprint differs from the library "
            "because its 0.2 mm GND-pad vias were widened to JLCPCB's 0.3 mm minimum; the power "
            "switch's lever silk overhangs the edge by design; and a few GND pads join the pour "
            "with one thermal spoke but also have their own via.", SMALL),
          p("Net classes", H3),
          table([["Class", "Track", "Clearance", "Nets"],
                 ["Battery", "0.4 mm", "0.15 mm", "VBUS, VSYS, BATT_P, BATT_N (≈ 7 °C rise at 1 A)"],
                 ["Power", "0.4 mm", "0.15 mm", "+3V3, GND"],
                 ["Analog", "0.25 mm", "0.2 mm", "+3V3A, STICK_*, VBAT_SENSE, VPOT_SENSE"],
                 ["USB", "0.3 mm", "0.15 mm", "USB_DP / USB_DN, short and coupled"],
                 ["Default", "0.2 mm", "0.15 mm", "everything else"]],
                [22, 18, 20, 110]),
          p("0.4 mm, not wider, for the battery path: wider tracks could not enter the charger's "
            "0.5 mm-pitch QFN pads, and 0.4 mm on 1 oz copper carries 1 A at ≈ 7 °C rise (IPC-2221).",
            SMALL),
          Spacer(1, 4),
          image(rc_top, 165), caption("Figure 3 – front: controls only (through-hole)"),
          image(rc_bot, 165), caption("Figure 4 – back: all SMD parts, 18650 holder, USB-C"),
          PageBreak()]

    s += [p("2.6  Manufacturing (JLCPCB, full assembly)", H2),
          p("<font name='DejaVuMono'>controller/gen_fab.py</font> writes the order files to "
            "<font name='DejaVuMono'>controller/build/jlcpcb/</font>: the Gerber + drill zip, "
            "the BOM with an LCSC number per line and the CPL (placement) file. SMD parts are "
            "reflowed on the back; the through-hole controls on the front are soldered by "
            "JLCPCB's THT service. Only J2 (debug UART header) is not fitted, plus the four "
            "mounting holes, which are not parts. "
            "<font name='DejaVuMono'>gen_fab.py --stock</font> re-checks live stock."),
          table(bom_rows, [9, 46, 66, 20, 29]),
          Spacer(1, 4),
          KeepTogether([p("Ordering notes", H3), *bullets([
              "All resistors and MLCCs are JLCPCB Basic parts (no per-part setup fee).",
              "SW9 (C&amp;K 7103SYCQE) has no LCSC stock: choose it under Global Sourcing on the "
              "BOM page, or consign it. SW8 (7101SYCQE) had only 2 in stock on 2026-10-21.",
              "The CPL uses the kicad-jlcpcb-tools conventions (pad-centre position, mirrored "
              "bottom angles) and JLCKicadTools' rotation corrections. Check U2, U5, J1 and Q1 "
              "in JLCPCB's placement preview before paying.",
              "Low stock to watch: CPT-9019S buzzer, Keystone 1042 holder."])]),
          PageBreak()]

    # ---- 3 flight controller
    fc_drc = ", ".join(f"{v} {k.replace('_', ' ')}" for k, v in sorted(fc_chk["drc"].items())
                       if k != "unconnected_items") or "none"
    s += [p("3  Flight controller (falcon_fc)", H1),
          p("A 45 × 45 mm, 4-layer board on the standard 30.5 mm stack pattern (4 mm holes for "
            "M3 rubber grommets). It replaces the Nucleo-64 + breakout bench setup: MCU, IMU, "
            "barometer, GPS, radio socket, ESC and AUX connectors on one board, with the pin "
            "assignment kept to the existing CubeMX project (falconDrone.ioc) wherever possible."),
          table([["Block", "Part", "Notes"],
                 ["MCU", "STM32F446RET6 (LQFP-64)", "one 100 nF per VDD pin, low-ESR ceramic on VCAP_1, "
                  "SWD + SWO on a JST-GH that matches Nucleo CN4"],
                 ["IMU", "TDK ICM-42688-P", "SPI3 ≤ 24 MHz, INT1 to PB5, near the centre of mass"],
                 ["Barometer", "MS5611-01BA03", "I2C1 at 0x77, away from the regulators, cover the "
                  "port with open-cell foam"],
                 ["GPS", "u-blox SAM-M10Q", "USART6, own LP5907 LDO, solid GND under the patch antenna"],
                 ["Radio", "nRF24L01+ module socket", "SPI2, module overhangs the south edge"],
                 ["Power in", "JST-GH: VBAT, GND, +5 V", "AO3401A reverse-polarity P-FET, SMAJ5.0A TVS"],
                 ["3.3 V", "AP7361C-33E (SOT-223)", "main rail, ≈ 0.3 W worst case into a GND pour"],
                 ["Sensor 3.3 V", "LP5907MFX-3.3", "quiet rail for IMU + baro (+3V3_SENS)"],
                 ["Battery sense", "10k / 1k divider", "6S 25.2 V → 2.29 V on PA4"],
                 ["ESD", "3 × SP0504BAHT", "at the ESC, AUX and telemetry connectors"]],
                [24, 46, 100]),
          p("3.1  Pin map", H2),
          table([["Function", "Pins", "Function", "Pins"],
                 ["ESC M1–M4", "PA0 PA1 (TIM2) · PA6 PA7 (TIM3)", "IMU SPI3", "PC10 SCK · PC11 MISO · PC12 MOSI"],
                 ["nRF24 SPI2", "PB13 SCK · PB14 MISO · PB15 MOSI", "IMU CS / INT1", "PA15 / PB5"],
                 ["nRF24 CSN / CE / IRQ", "PB12 / PB1 / PC4", "Baro I2C1", "PB6 SCL · PB7 SDA"],
                 ["Telemetry USART2", "PA2 TX · PA3 RX", "GPS USART6", "PC6 TX · PC7 RX"],
                 ["AUX USART1 + I2C3", "PA9 PA10 · PA8 PC9", "VBAT_SENSE", "PA4 (ADC)"],
                 ["SWD", "PA13 · PA14 · PB3 SWO", "LED / CAL button", "PA5 / PC13"]],
                [32, 53, 30, 55], mono_cols=(1, 3)),
          p("New pins, not yet in the .ioc: SPI3 IMU, I2C1 baro, PA4 VBAT_SENSE and PC4 NRF_IRQ. "
            "The RC link on this board is the nRF24 only (build with FALCON_RC_RADIO=NRF24).", SMALL),
          p("3.2  PCB", H2),
          table([["Item", "Value"],
                 ["Stack-up", "F.Cu signals · In1 GND plane · In2 +3V3 plane · B.Cu signals, 1.6 mm"],
                 ["Placement", "top: GPS centre, power entry north, ESC header east, AUX west, nRF24 "
                  "south; bottom: MCU under the GPS (shielded by In1), IMU near the centre, baro apart"],
                 ["Routing", f"{fc['tracks']} track segments, {fc['vias']} vias; maze-routed IMU bus "
                  "and local links first, then Freerouting in stages"],
                 ["Checks", f"ERC {fc_chk['erc']} violations · {fc_chk['unconnected']} unconnected · "
                  f"{fc_chk['parity']} schematic-parity issues · DRC remaining: {fc_drc} "
                  "(the nRF24 footprint was edited on purpose)"]],
                [28, 142]),
          Spacer(1, 4),
          Table([[[image(fc_top, 52), caption("Figure 5 – FC top")],
                  [image(fc_bot, 52), caption("Figure 6 – FC bottom")]]],
                colWidths=[80 * mm, 80 * mm]),
          PageBreak()]

    # ---- 4 pipeline
    s += [p("4  Design pipeline", H1),
          p("Everything is generated from Python, so every change is a reviewable diff and the "
            "boards can be rebuilt from scratch. KiCad is used to view, check and export. "
            "Hand edits to the .kicad_* files are lost the next time a script runs."),
          table([["Step", "Script", "Output"],
                 ["Shared helpers", "hardware/schgen.py", "label-only schematic writer, symbol library "
                  "handling, R/C/header helpers"],
                 ["Schematic", "gen_schematic.py", "*.kicad_sch + project symbols; then "
                  "kicad-cli sch erc"],
                 ["Footprints", "controller/gen_footprints.py", "falcon_rc.pretty (Alps stick, 4.7 mm toggle)"],
                 ["Placement", "gen_pcb.py [--force]", "outline, rules, net classes, placed parts, pours"],
                 ["Routing", "route_pcb.py", "GND fan-out → Freerouting → maze finisher → pours → "
                  "stitching vias → GND-island join"],
                 ["Order files", "controller/gen_fab.py, flight_controller/export_fab.py",
                  "Gerbers + drill zip, BOM, CPL in build/jlcpcb/"],
                 ["This document", "hardware/gen_design_doc.py", "FalconDrone_hardware_design.pdf"]],
                [26, 58, 86], mono_cols=(1,)),
          Spacer(1, 4),
          p("route_pcb.py needs Java 25+ and the Freerouting 2.x jar: "
            "<font name='DejaVuMono'>FREEROUTING_JAR=… JAVA=… python3 route_pcb.py --force</font>. "
            "build/ directories are git-ignored.", SMALL),
          p("5  Verification status and open items", H1),
          p("Verified", H3),
          *bullets([
              "Charger, protection, LDO, ESD, buzzer driver and FET pinouts checked against the "
              "datasheets (TI SLUS810N, Fortune/HXY 8205A, CUI CPT-9019S, C&amp;K 7000 series).",
              "Pin-by-pin netlist audit of the controller; ERC clean on both schematics; both "
              "boards DRC clean with schematic parity.",
              "Every controller part matched to a stocked JLCPCB part (one Global Sourcing)."]),
          p("Open before / after the first build", H3),
          *bullets([
              "Controller firmware: retarget from ESP32 to ESP32-S3 with a new board.h for the "
              "pin map in 2.3.",
              "Print the Alps stick footprint 1:1 and test-fit a real stick (the switch-row "
              "position was measured off the Alps drawing, not dimensioned).",
              "Check part rotations in JLCPCB's placement preview (U2, U5, J1, Q1).",
              "1.07 A input limit can overload a USB-2 port that only allows 500 mA; phone and "
              "USB-C chargers are fine.",
              "Check the OLED module's pin order (GND/VCC) before plugging it in.",
              "Flight controller: add the new pins (SPI3, I2C1, PA4, PC4) to falconDrone.ioc."]),
          ]

    body = os.path.join(BUILD, "body.pdf")

    def decorate(canvas, doc):
        canvas.saveState()
        canvas.setFont("DejaVu", 7)
        canvas.setFillColor(MUTED)
        if doc.page > 1:
            canvas.drawString(18 * mm, 10 * mm, "FalconDrone hardware design · rev A")
            canvas.drawRightString(A4[0] - 18 * mm, 10 * mm, f"page {doc.page}")
            canvas.setStrokeColor(RULE)
            canvas.line(18 * mm, 13 * mm, A4[0] - 18 * mm, 13 * mm)
        canvas.restoreState()

    doc = SimpleDocTemplate(body, pagesize=A4, leftMargin=20 * mm, rightMargin=20 * mm,
                            topMargin=16 * mm, bottomMargin=18 * mm,
                            title="FalconDrone hardware design", author="FalconDrone")
    doc.build(s, onFirstPage=decorate, onLaterPages=decorate)

    # appendix: both schematics, as KiCad plots them
    sheets = []
    for base in (RC, FC):
        pdf = os.path.join(BUILD, os.path.basename(base) + "_sch.pdf")
        run(["kicad-cli", "sch", "export", "pdf", "-o", pdf, base + ".kicad_sch"])
        sheets.append(pdf)
    run(["pdfunite", body, *sheets, OUT])
    print(f"wrote {OUT}")


if __name__ == "__main__":
    build()
