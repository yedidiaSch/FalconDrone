#!/usr/bin/env python3
"""
Create falcon_carrier.kicad_pcb: board outline, 2-layer stackup, JLCPCB rules,
GND pours, mounting holes and the part placement.

Board: 70 x 81 mm, the Nucleo-F446RE's width. Coordinates: x 50..120,
y 50..131, north (y = 50) is the Nucleo's ST-LINK end.
  North 57 mm : under the Nucleo (target part, UM1724 fig. 5). The two 2x19
                morpho sockets, the IMU in the middle with its LDO, level
                translator and pull-ups, battery sense and buzzer driver.
                Everything here stays under ~6 mm: the Nucleo sits on 8.5 mm
                sockets.
  South 24 mm : what is too tall or must not be under the Nucleo. Battery
                sense pads, BEC 5 V input + bulk cap, ESC header, I2C test
                header, buzzer, and the nRF24 socket with the module's
                antenna past the east edge.
  Holes       : H1-H4 M3 in the corners for the frame (soft-mount),
                H5-H7 the Nucleo's own three M3 holes for standoffs.
Layers : F.Cu signals + GND pour | B.Cu signals + GND pour.

Nucleo geometry (UM1724 fig. 5, top view), relative to CN7 pin 1:
  CN7 odd pins x 0, even 2.54; CN10 odd 60.96, even 63.50; rows 2.54 apart
  Board edge x -3.25 .. 66.75, bottom edge row 19 + 3.04
  Holes (7.62, -3.81) (55.88, -2.54) (40.64, 48.26), the Arduino Uno holes

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

PROJECT = "falcon_carrier"
FP_DIR = "/usr/share/kicad/footprints"
X0, Y0, W, H = 50.0, 50.0, 70.0, 81.0
CORNER_R = 3.0
HOLE_INSET = 3.5               # frame hole centre from each edge

NX, NY = X0 + 3.25, Y0 + 8.78  # Nucleo CN7 pin 1
NUCLEO_BOTTOM = NY + 18 * 2.54 + 3.04

mm = pcbnew.FromMM

# ref: (x, y, rotation_deg, side). Coordinates in mm, KiCad frame (y down),
# as seen from the top. Rotation is counter-clockwise as shown in KiCad.
PLACEMENT = {
    # Nucleo sockets: the pin header footprint, pin 1 at its origin
    "J1": (NX, NY, 0, "top"),
    "J2": (NX + 60.96, NY, 0, "top"),
    "H5": (NX + 7.62, NY - 3.81, 0, "top"),
    "H6": (NX + 55.88, NY - 2.54, 0, "top"),
    "H7": (NX + 40.64, NY + 48.26, 0, "top"),

    # IMU in the middle of the board, translator north of it, LDO west
    "U3": (85.0, 86.0, 0, "top"),
    "C6": (89.2, 87.0, 90, "top"),       # VDD, pin 13 east side
    # VDDIO (pin 8) and REGOUT (pin 10) caps to either side, so the GND pins
    # between them (9, 11) can drop their vias straight south
    "C7": (81.0, 89.5, 0, "top"),
    "C8": (89.0, 90.5, 0, "top"),
    "U2": (77.0, 88.0, 0, "top"),
    "C4": (74.0, 92.0, 90, "top"),
    "C5": (78.5, 93.5, 90, "top"),
    "U4": (85.0, 76.0, 0, "top"),
    "R4": (78.5, 74.5, 90, "top"),
    "R5": (78.5, 79.0, 90, "top"),
    "R6": (91.5, 74.5, 90, "top"),
    "R7": (91.5, 79.0, 90, "top"),
    "R3": (95.0, 71.0, 0, "top"),
    "C9": (95.0, 74.5, 90, "top"),

    # battery sense and buzzer driver, near PA4 / PB0 (CN7 rows 16, 17)
    "R1": (62.0, 104.0, 0, "top"),
    "R2": (62.0, 100.5, 0, "top"),
    "C3": (66.5, 97.0, 0, "top"),
    "Q1": (99.5, 101.0, 0, "top"),
    "R8": (86.0, 102.0, 0, "top"),
    "R9": (86.0, 98.5, 0, "top"),

    # south strip: battery sense pads, I2C test + 5 V cap, ESC + BEC, buzzer
    "J3": (53.5, 111.0, 270, "top"),     # pads run south
    "J4": (59.0, 113.0, 90, "top"),      # pins run east
    "C2": (72.0, 119.5, 0, "top"),
    "J6": (59.0, 124.0, 90, "top"),
    "J7": (73.0, 124.0, 90, "top"),
    "BZ1": (82.5, 119.0, 0, "top"),
    # nRF24: pins on the board, antenna past the east edge
    "J5": (104.0, 110.3, 0, "top"),
    "C10": (100.0, 112.0, 90, "top"),
    "C11": (100.0, 116.0, 90, "top"),
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
    ("Default", 0.25, 0.2, 0.6, 0.3, []),
    # battery and 5 V: <= 0.5 A
    ("Power", 0.5, 0.2, 0.8, 0.4, ["VBAT_IN", "VBAT", "+5V", "+3V3", "GND"]),
    ("Analog", 0.25, 0.2, 0.6, 0.3, ["VBAT_SENSE"]),
    # the ICM-20948's pads are 0.4 mm pitch, 0.2 mm wide: 0.15 / 0.13 fits between them
    ("IMU", 0.15, 0.13, 0.6, 0.3, ["+1V8", "IMU_*"]),
]


def fp_dir(lib):
    return f"{FP_DIR}/{lib}.pretty"


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


def add_line(board, a, b, layer=pcbnew.F_SilkS, width=0.15):
    s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
    s.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
    s.SetLayer(layer)
    s.SetWidth(mm(width))
    board.Add(s)


def add_silkscreen(board):
    """Labels for the person soldering and wiring the board."""
    # The nRF24 footprint outlines the whole module, which overhangs the east
    # edge; drop the silk that would land off the board.
    j5 = board.FindFootprintByReference("J5")
    for item in list(j5.GraphicalItems()):
        if item.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS) and \
                pcbnew.ToMM(item.GetBoundingBox().GetRight()) > X0 + W - 0.3:
            j5.Remove(item)
    for fp in board.Footprints():
        ref = fp.GetReference()
        if ref.startswith("H") or ref in ("J3", "J4", "J5", "J6", "J7"):
            fp.Reference().SetVisible(False)       # the board labels below name them
        elif ref.startswith("R") and fp.Pads()[0].GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
            # axial / disc parts: reference on the body, clear of the neighbours
            p1, p2 = (pad.GetPosition() for pad in fp.Pads())
            fp.Reference().SetPosition(pcbnew.VECTOR2I((p1.x + p2.x) // 2, (p1.y + p2.y) // 2))
            fp.Reference().SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
            fp.Reference().SetTextThickness(mm(0.15))
    # where the Nucleo's south edge lands (everything north of it must stay low)
    y = NUCLEO_BOTTOM
    for x in range(int(X0) + 1, int(X0 + W) - 1, 4):
        add_line(board, (x, y), (x + 2, y))
    add_text(board, "NUCLEO-F446RE  (ST-LINK end north)", 85.0, 52.5, size=0.9)
    add_text(board, "max 6 mm under the Nucleo", 72.0, NUCLEO_BOTTOM - 1.2, size=0.8)
    add_text(board, "set Nucleo JP5 to E5V", 85.0, 55.0, size=0.8)
    add_text(board, "- VBAT +", 56.6, 113.4, angle=90, size=0.8)
    add_text(board, "3V3 SCL SDA G", 62.8, 115.6, size=0.8)
    add_text(board, "M1 M2 M3 M4 G", 64.1, 126.4, size=0.8)
    add_text(board, "BEC+ G", 74.3, 126.4, size=0.8)
    add_text(board, "nRF24", 109.0, 123.0, size=0.8)
    add_text(board, "FalconDrone carrier rev A", 86.0, 109.0, size=0.8)


def place(fp, x, y, rot, side):
    if side == "bottom":
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
        rot += 180
    fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    fp.SetOrientationDegrees(rot % 360)


def patch_project_rules():
    path = os.path.join(HERE, PROJECT + ".kicad_pro")
    pro = json.load(open(path)) if os.path.exists(path) else {
        "meta": {"filename": PROJECT + ".kicad_pro", "version": 3}}
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
        if ref == "U4":
            fp.SetLocalClearance(mm(0.13))   # VSSOP-8 0.5 mm pitch: its own pads sit 0.15 apart
        if ref in ("J1", "J2"):
            # female sockets go in these holes: show one in the 3D view. KiCad's
            # socket model is mirrored (pin 2 west of pin 1), hence the offset.
            model = fp.Models()[0]
            model.m_Filename = ("${KICAD9_3DMODEL_DIR}/Connector_PinSocket_2.54mm.3dshapes/"
                                "PinSocket_2x19_P2.54mm_Vertical.step")
            model.m_Offset = pcbnew.VECTOR3D(2.54, 0, 0)
        if ref == "J3":
            fp.SetExcludedFromBOM(False)   # the wire-pad footprint says so; it is a real part here
        if ref in ("H5", "H6", "H7"):
            # the Nucleo's own holes: its socket pins already sit this close
            for item in list(fp.GraphicalItems()):
                if item.GetLayer() in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
                    fp.Remove(item)
        if ref in PLACEMENT:
            place(fp, *PLACEMENT[ref])
        elif ref.startswith("H") and ref[1:].isdigit() and int(ref[1:]) <= 4:
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
