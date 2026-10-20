#!/usr/bin/env python3
"""
Create falcon_rc.kicad_pcb: board outline, 2-layer stackup, JLCPCB rules,
GND pours, mounting holes and the initial part placement.

This is a one-shot starting point, not a round-trip generator: once routing
starts in KiCad, edit the .kicad_pcb directly. Rerunning this script
overwrites the board (it refuses to if the board already has tracks).

Board: 150 x 100 mm, 6 mm corner radius, held in both hands like a gamepad.
Coordinates: x 50..200, y 50..150, north (y = 50) is the edge facing away
from the pilot.
  Top    : sticks in the lower corners (80 mm apart), OLED module top centre
           over its socket, D-pad style menu buttons between the sticks,
           ARM toggle top-left, MODE toggle top-right, nRF24 module lying
           north-east with its antenna past the north edge. Through-hole only.
  Bottom : every SMD part (one assembly pass). USB-C on the north edge,
           power slide switch on the west edge,
           ESP32-S3 module north-west with its antenna on the north edge
           (nothing else in its keepout), charger + LDO north-centre next to
           USB, 18650 holder across the south half, protection at its minus
           end, buzzer south-east.
Layers : F.Cu signals + GND pour | B.Cu signals + GND pour.

Usage:  python3 gen_pcb.py [--force]
"""

import json
import math
import os
import subprocess
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import schgen as sch  # noqa: E402  (reuses the s-expression parser)

PROJECT = "falcon_rc"
FP_DIR = "/usr/share/kicad/footprints"
X0, Y0, W, H = 50.0, 50.0, 150.0, 100.0
CORNER_R = 6.0
HOLE_INSET = 5.0               # mounting hole centre from each edge

mm = pcbnew.FromMM

# ref: (x, y, rotation_deg, side). Coordinates in mm, KiCad frame (y down),
# as seen from the top. Rotation is counter-clockwise as shown in KiCad.
# Tactile switches have their origin on pin 1, the button centre is +3.25/+2.25.
PLACEMENT = {
    # ---------------- top: controls ----------------
    "JS1": (85.0, 108.0, 0, "top"),        # left stick: throttle / yaw
    "JS2": (165.0, 108.0, 0, "top"),       # right stick: pitch / roll
    "SW4": (121.75, 95.75, 0, "top"),      # UP
    "SW5": (121.75, 117.75, 0, "top"),     # DOWN
    "SW6": (132.75, 106.75, 0, "top"),     # OK (right)
    "SW7": (110.75, 106.75, 0, "top"),     # BACK (left)
    "SW8": (66.0, 64.0, 0, "top"),         # ARM toggle
    "SW9": (184.0, 64.0, 0, "top"),        # MODE toggle
    # OLED module on a socket, pins run east; the module hangs south over
    # y 60..87 (SMD parts under it are fine, the socket is 8.5 mm tall)
    "J4": (121.19, 62.0, 90, "top"),
    "D4": (147.5, 54.5, 0, "top"),         # status LED
    # 3 mm LEDs; origin on pad 1 (K), body centre +1.27
    "D1": (137.5, 54.5, 0, "top"),         # PWR (green)
    "D2": (142.5, 54.5, 0, "top"),         # CHG (red)
    # nRF24 socket, the module lies north over y 54..84, its SMA past the edge
    "J3": (160.0, 82.0, 90, "top"),

    # ---------------- bottom: every SMD part, so one assembly pass ----------------
    # USB-C, mouth out of the north edge, ESD + CC resistors right behind it
    "J1": (125.0, 53.2, 180, "bottom"),
    "U1": (133.0, 55.5, 90, "bottom"),     # east of J1: west of it is antenna keepout
    "R1": (119.0, 59.5, 0, "bottom"),
    "R2": (131.0, 59.5, 0, "bottom"),
    # power slide, lever out of the west edge (90 here: bottom parts mirror east-west)
    "SW1": (53.0, 85.0, 90, "bottom"),

    "U5": (95.0, 64.5, 0, "bottom"),       # antenna on the north edge
    # east of the module (its pins 2-14: 3V3, EN, ADC, USB)
    "C14": (107.4, 60.4, 0, "bottom"),
    "R17": (110.4, 60.4, 0, "bottom"),
    "C15": (113.4, 60.4, 0, "bottom"),
    "C13": (114.0, 63.0, 0, "bottom"),     # 10u for pin 2
    "C20": (107.4, 63.0, 0, "bottom"),     # STICK_RX (IO4)
    "C21": (110.4, 63.0, 0, "bottom"),     # STICK_RY (IO5)
    "R13": (107.4, 65.6, 0, "bottom"),     # VBAT divider (IO6)
    "R14": (110.4, 65.6, 0, "bottom"),
    "C11": (113.4, 65.6, 0, "bottom"),
    "R11": (107.4, 68.2, 0, "bottom"),     # VPOT divider (IO7)
    "R12": (110.4, 68.2, 0, "bottom"),
    "C10": (113.4, 68.2, 0, "bottom"),
    "R19": (113.4, 71.0, 0, "bottom"),     # I2C pull-ups
    "R20": (116.4, 71.0, 0, "bottom"),
    # south of the module (pins 15-26: nRF SPI, I2C SCL, VBUS_DET, buzzer, LED)
    "R15": (103.0, 81.0, 90, "bottom"),    # VBUS_DET divider (IO3)
    "R16": (105.0, 81.0, 90, "bottom"),
    "C12": (107.0, 81.0, 90, "bottom"),
    "SW2": (88.0, 83.0, 0, "bottom"),      # RESET
    "SW3": (96.0, 83.0, 0, "bottom"),      # BOOT
    "R18": (92.0, 79.5, 0, "bottom"),
    # west of the module (pins 27-40: IO0, IO1/IO2, UART, switches)
    "C18": (83.0, 60.0, 90, "bottom"),     # STICK_LX (IO1)
    "C19": (83.0, 63.0, 90, "bottom"),     # STICK_LY (IO2)
    "J2": (79.5, 66.0, 0, "bottom"),       # debug UART, pins run south
    # stick supply filter, between the module and the sticks
    "FB1": (112.0, 86.0, 0, "bottom"),
    "C8": (115.5, 86.0, 90, "bottom"),
    "C9": (117.5, 86.0, 90, "bottom"),

    # ---------------- bottom: power ----------------
    # charger, behind the USB-C
    "U2": (134.0, 66.0, 0, "bottom"),
    "C1": (134.0, 61.6, 0, "bottom"),
    "C2": (139.5, 62.0, 0, "bottom"),
    "C3": (139.5, 69.5, 0, "bottom"),
    "R3": (128.5, 68.5, 0, "bottom"),
    "R4": (128.5, 71.0, 0, "bottom"),
    "R5": (131.5, 71.0, 0, "bottom"),
    "R24": (134.5, 71.0, 0, "bottom"),
    "R6": (139.0, 57.5, 90, "bottom"),     # under D1/D2
    "R7": (144.0, 57.5, 90, "bottom"),
    # 3V3 LDO
    "U4": (147.5, 66.0, 0, "bottom"),
    "C5": (143.8, 62.0, 90, "bottom"),
    "C6": (151.2, 62.0, 90, "bottom"),
    "C7": (151.2, 69.0, 90, "bottom"),
    "R10": (147.5, 70.5, 0, "bottom"),
    # nRF24 supply caps at the socket's VCC pin
    "C16": (156.5, 80.5, 90, "bottom"),
    "C17": (154.5, 80.5, 90, "bottom"),
    # 18650 holder: pad 1 (+) lands east once flipped, pad 2 (-) west
    "BT1": (125.0, 137.5, 0, "bottom"),
    # protection at the minus end, north of the holder
    "Q1": (98.0, 121.0, 0, "bottom"),
    "U3": (104.0, 121.0, 0, "bottom"),
    "R8": (108.0, 119.5, 0, "bottom"),
    "C4": (108.0, 122.5, 0, "bottom"),
    "R9": (92.5, 121.0, 90, "bottom"),
    # buzzer + driver, south-east
    "BZ1": (182.0, 131.0, 0, "bottom"),
    "Q2": (177.0, 122.0, 0, "bottom"),
    "R21": (173.5, 122.0, 90, "bottom"),
    "R22": (181.0, 122.0, 90, "bottom"),
    "R25": (185.5, 122.0, 90, "bottom"),  # piezo bleed
    "R23": (148.8, 57.5, 90, "bottom"),    # under D4
}

# JLCPCB 2-layer standard capabilities (no extra-cost options), with margin.
RULES = {
    "min_clearance": 0.127,
    "min_track_width": 0.127,
    "min_via_diameter": 0.45,
    "min_through_hole_diameter": 0.3,
    "min_via_annular_width": 0.075,
    "min_hole_to_hole": 0.25,
    "min_copper_edge_clearance": 0.3,
    "min_silk_clearance": 0.0,
    "min_text_height": 0.8,
    "min_text_thickness": 0.15,
}
NETCLASSES = [
    # name, track, clearance, via dia, via drill, net patterns
    ("Default", 0.2, 0.15, 0.6, 0.3, []),
    # charge / battery path, up to ~1 A: 0.4 mm on 1 oz is ~7 C rise at 1 A
    # (IPC-2221), and still fits the BQ24073's 0.5 mm-pitch pads at 0.15 mm
    # clearance (0.5 - 0.2 - 0.125 = 0.175).
    ("Battery", 0.4, 0.15, 0.8, 0.4, ["VBUS", "VSYS", "BATT_P", "BATT_N"]),
    ("Power", 0.4, 0.15, 0.8, 0.4, ["+3V3", "GND"]),
    ("Analog", 0.25, 0.2, 0.6, 0.3, ["+3V3A", "STICK_*", "VBAT_SENSE", "VPOT_SENSE"]),
    # 90 ohm pair on 1.6 mm 2-layer is unreachable; keep it short and coupled
    ("USB", 0.3, 0.15, 0.6, 0.3, ["USB_DP", "USB_DN"]),
]


def fp_dir(lib):
    return os.path.join(HERE, lib + ".pretty") if lib == PROJECT else f"{FP_DIR}/{lib}.pretty"


def load_netlist():
    path = os.path.join(HERE, "build", PROJECT + ".net")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    subprocess.run(["kicad-cli", "sch", "export", "netlist", "-o", path,
                    os.path.join(HERE, PROJECT + ".kicad_sch")], check=True, capture_output=True)
    t = sch.parse_sexp(open(path).read())
    comps = {}
    for c in sch.find(sch.find1(t, "components"), "comp"):
        dnp = any(str(sch.find1(prop, "name")[1]) == "dnp" for prop in sch.find(c, "property"))
        comps[str(sch.find1(c, "ref")[1])] = (str(sch.find1(c, "value")[1]),
                                              str(sch.find1(c, "footprint")[1]), dnp)
    nets = {}
    for n in sch.find(sch.find1(t, "nets"), "net"):
        name = str(sch.find1(n, "name")[1])           # local labels come as "/NAME"
        nets[name] = [(str(sch.find1(x, "ref")[1]), str(sch.find1(x, "pin")[1]))
                      for x in sch.find(n, "node")]
    return comps, nets


def outline_points(inset=0.0, arc_steps=8):
    """Rounded rectangle as a polygon (for zones)."""
    r = CORNER_R - inset
    x0, y0, x1, y1 = X0 + inset, Y0 + inset, X0 + W - inset, Y0 + H - inset
    pts = []
    for cx, cy, a0 in ((x1 - r, y0 + r, 270), (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)):
        for k in range(arc_steps + 1):
            a = math.radians(a0 + 90 * k / arc_steps)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def add_outline(board):
    x0, y0, x1, y1 = X0, Y0, X0 + W, Y0 + H
    r = CORNER_R

    def seg(a, b):
        s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
        s.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.05))
        board.Add(s)

    def arc(cx, cy, start_deg):
        s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_ARC)
        s.SetCenter(pcbnew.VECTOR2I(mm(cx), mm(cy)))
        a = math.radians(start_deg)
        s.SetStart(pcbnew.VECTOR2I(mm(cx + r * math.cos(a)), mm(cy + r * math.sin(a))))
        s.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(90, pcbnew.DEGREES_T), True)
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.05))
        board.Add(s)

    seg((x0 + r, y0), (x1 - r, y0))
    seg((x1, y0 + r), (x1, y1 - r))
    seg((x1 - r, y1), (x0 + r, y1))
    seg((x0, y1 - r), (x0, y0 + r))
    arc(x1 - r, y0 + r, 270)
    arc(x1 - r, y1 - r, 0)
    arc(x0 + r, y1 - r, 90)
    arc(x0 + r, y0 + r, 180)


def add_zone(board, layer, net, priority=0):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net)
    z.SetAssignedPriority(priority)
    z.SetMinThickness(mm(0.2))
    z.SetLocalClearance(mm(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.3))
    z.SetThermalReliefSpokeWidth(mm(0.4))
    ol = z.Outline()
    ol.NewOutline()
    for x, y in outline_points(0.3):
        ol.Append(mm(x), mm(y))
    board.Add(z)
    return z


def add_text(board, text, x, y, bottom=False, angle=0, size=1.0):
    txt = pcbnew.PCB_TEXT(board)
    txt.SetText(text)
    txt.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    txt.SetLayer(pcbnew.B_SilkS if bottom else pcbnew.F_SilkS)
    txt.SetMirrored(bottom)
    txt.SetTextAngleDegrees(angle)
    txt.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    txt.SetTextThickness(mm(0.15 if size < 1.2 else 0.25))
    board.Add(txt)


def add_silkscreen(board):
    """Labels for the person holding the board; JLCPCB places parts from the CPL,
    so passives lose their reference text."""
    for fp in board.Footprints():
        # passives, plus the parts that get a function label below
        if fp.GetReference().startswith(("R", "C", "FB", "D", "Q", "SW", "JS", "BT")):
            fp.Reference().SetVisible(False)
    # The nRF24 socket's footprint outlines the whole module, which overhangs the
    # north edge; drop the silk that would land off the board.
    j3 = board.FindFootprintByReference("J3")
    for item in list(j3.GraphicalItems()):
        if item.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS) and \
                pcbnew.ToMM(item.GetBoundingBox().GetTop()) < Y0 + 0.3:
            j3.Remove(item)
    add_text(board, "ARM", 66.0, 70.0)
    add_text(board, "MODE", 184.0, 70.0)
    add_text(board, "ANGLE  RATE  -", 184.0, 58.0, size=0.8)
    add_text(board, "PWR", 57.5, 85.0, angle=90)
    add_text(board, "UP", 125.0, 93.2, size=0.8)
    add_text(board, "DOWN", 125.0, 126.2, size=0.8)
    add_text(board, "BACK", 108.0, 109.0, angle=90, size=0.8)
    add_text(board, "OK", 141.8, 109.0, angle=90, size=0.8)
    add_text(board, "THR / YAW", 85.0, 123.5, size=0.8)
    add_text(board, "PITCH / ROLL", 165.0, 123.5, size=0.8)
    add_text(board, "PWR  CHG   ST", 143.0, 51.7, size=0.8)
    add_text(board, "FalconDrone controller rev A", 125.0, 146.5)
    # back: battery polarity, big (a reversed cell is not protected)
    # mirrored text: the first character ends up at the east end, over pad 1 (+)
    add_text(board, "+   18650   -", 125.0, 124.6, bottom=True, size=1.6)
    add_text(board, "UART: G TX RX 3V3", 79.5, 77.0, bottom=True, size=0.8)
    add_text(board, "RST", 88.0, 87.0, bottom=True, size=0.8)
    add_text(board, "BOOT", 96.0, 87.0, bottom=True, size=0.8)


def place(fp, x, y, rot, side):
    """Bottom parts are mirrored east-west as seen from the top: a part that
    faces north on top still faces north on the bottom. (Flip + orientation 0
    would mirror it north-south, hence the extra 180.)"""
    if side == "bottom":
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
        rot += 180
    fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    fp.SetOrientationDegrees(rot % 360)


def patch_project_rules():
    path = os.path.join(HERE, PROJECT + ".kicad_pro")
    pro = json.load(open(path))
    ds = pro.setdefault("board", {}).setdefault("design_settings", {})
    ds.setdefault("rules", {}).update(RULES)
    ns = pro.setdefault("net_settings", {})
    ns["classes"] = [{
        "name": name, "track_width": tw, "clearance": cl, "via_diameter": vd, "via_drill": vdr,
        "microvia_diameter": 0.3, "microvia_drill": 0.1, "diff_pair_width": 0.3,
        "diff_pair_gap": 0.15, "diff_pair_via_gap": 0.25, "wire_width": 6, "bus_width": 12,
        "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
        "schematic_color": "rgba(0, 0, 0, 0.000)", "priority": 2147483647 if name == "Default" else i,
    } for i, (name, tw, cl, vd, vdr, _) in enumerate(NETCLASSES)]
    ns["netclass_patterns"] = [{"netclass": name, "pattern": "/" + pat}
                               for name, *_, pats in NETCLASSES for pat in pats]
    ns.setdefault("meta", {"version": 4})
    with open(path, "w") as f:
        json.dump(pro, f, indent=2)
        f.write("\n")


def main():
    out = os.path.join(HERE, PROJECT + ".kicad_pcb")
    if os.path.exists(out) and "--force" not in sys.argv:
        old = pcbnew.LoadBoard(out)
        if len(old.GetTracks()):
            raise SystemExit(f"{out} already has tracks; edit it in KiCad (or pass --force)")

    comps, nets = load_netlist()
    board = pcbnew.CreateEmptyBoard()
    board.SetCopperLayerCount(2)
    board.GetDesignSettings().SetBoardThickness(mm(1.6))

    netinfo = {}
    for name in nets:
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        netinfo[name] = ni
    pad_net = {(r, p): name for name, nodes in nets.items() for r, p in nodes}

    add_outline(board)

    holes = [(X0 + HOLE_INSET, Y0 + HOLE_INSET), (X0 + W - HOLE_INSET, Y0 + HOLE_INSET),
             (X0 + HOLE_INSET, Y0 + H - HOLE_INSET), (X0 + W - HOLE_INSET, Y0 + H - HOLE_INSET)]
    unplaced = []
    for ref, (value, fpid, dnp) in sorted(comps.items()):
        lib, name = fpid.split(":")
        fp = pcbnew.FootprintLoad(fp_dir(lib), name)
        fp.SetReference(ref)
        fp.SetValue(value)
        fp.SetFPID(pcbnew.LIB_ID(lib, name))
        fp.SetDNP(dnp)
        board.Add(fp)
        for pad in fp.Pads():
            net = pad_net.get((ref, pad.GetNumber()))
            if net:
                pad.SetNet(netinfo[net])
        if ref == "U5":
            # the module's GND pad vias are 0.2 mm; JLCPCB's standard minimum is 0.3
            for pad in fp.Pads():
                if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and pad.GetDrillSize().x < mm(0.3):
                    pad.SetDrillSize(pcbnew.VECTOR2I(mm(0.3), mm(0.3)))
                    pad.SetSize(pcbnew.F_Cu, pcbnew.VECTOR2I(mm(0.6), mm(0.6)))
        if ref in PLACEMENT:
            place(fp, *PLACEMENT[ref])
        elif ref.startswith("H") and ref[1:].isdigit():
            place(fp, *holes[int(ref[1:]) - 1], 0, "top")
        else:
            unplaced.append(ref)

    add_zone(board, pcbnew.F_Cu, netinfo["/GND"])
    add_zone(board, pcbnew.B_Cu, netinfo["/GND"])
    add_silkscreen(board)

    board.Save(out)
    # ZONE_FILLER segfaults on a board built in memory, but works once the
    # board has been loaded from disk.
    board = pcbnew.LoadBoard(out)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(out)
    patch_project_rules()
    print(f"wrote {out}: {len(comps)} parts" + (f", UNPLACED: {unplaced}" if unplaced else ""))


if __name__ == "__main__":
    main()
