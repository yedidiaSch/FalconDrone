#!/usr/bin/env python3
"""
Create rc_power.kicad_pcb: the RC's power board, 102 x 44 mm, 2 layers.

It mounts on the back of the RC carrier, top side facing out: the 18650
holder, the USB-C port, the fuel-gauge LEDs and the power button are all on
the outside. J2 (on the bottom, pins down) plugs into a 1x4 socket on the
RC carrier's back; four M3 standoffs (11 mm: 8.5 mm socket + 2.5 mm header)
hold it.

Seen from the RC's front the board is mirrored left-right: power-board x
maps to RC x = 202 - x, y to RC y = y + 48. The USB-C sits on this board's
east edge, which lands on the RC's west edge.

Coordinates: x 50..152, y 50..94.
  North : Keystone 1042 holder, + end (pad 1) east next to the IP5306.
  South : protection (DW01A + 8205A) west by the cell's minus end,
          LEDs + button centre, IP5306 + inductor east, USB-C east edge.

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

PROJECT = "rc_power"
FP_DIR = "/usr/share/kicad/footprints"
X0, Y0, W, H = 50.0, 50.0, 102.0, 44.0
CORNER_R = 3.0
HOLES = [(53.3, 53.5), (148.7, 53.5), (53.3, 90.5), (148.7, 90.5)]
RC_DX, RC_DY = 202.0, 48.0     # RC carrier x = RC_DX - x, y = y + RC_DY

mm = pcbnew.FromMM

# ref: (x, y, rotation_deg, side), as seen from this board's top.
PLACEMENT = {
    "BT1": (101.0, 63.0, 180, "top"),        # pad 1 (+) east
    # charger / boost, east
    "J1": (148.4, 80.2, 90, "top"),          # USB-C, opening east
    "R1": (141.0, 77.0, 90, "top"),
    "R2": (141.0, 82.0, 90, "top"),
    "U1": (130.0, 84.0, 0, "top"),
    "L1": (121.0, 80.0, 0, "top"),
    "C1": (136.5, 89.0, 0, "top"),           # VIN
    "R3": (141.0, 89.0, 90, "top"),
    "C2": (137.5, 92.0, 0, "top"),
    "C3": (125.5, 76.5, 0, "top"),           # BAT
    "R4": (116.0, 76.5, 0, "top"),
    "C4": (112.5, 79.0, 90, "top"),
    "C5": (125.0, 89.5, 0, "top"),           # VOUT
    "C6": (121.0, 89.5, 0, "top"),
    "C7": (117.0, 89.5, 0, "top"),
    # fuel gauge + button, centre
    "D1": (92.0, 89.0, 0, "top"),
    "D2": (96.0, 89.0, 0, "top"),
    "D3": (100.0, 89.0, 0, "top"),
    "D4": (104.0, 89.0, 0, "top"),
    "SW1": (98.0, 80.5, 0, "top"),
    "Q2": (107.5, 80.0, 0, "top"),
    "R5": (107.5, 84.0, 90, "top"),
    # protection, west by the minus pad
    "U2": (67.0, 80.0, 0, "top"),
    "R6": (62.0, 77.0, 90, "top"),
    "C8": (62.0, 82.0, 90, "top"),
    "R7": (67.0, 85.0, 0, "top"),
    "Q1": (75.0, 82.0, 0, "top"),
    # to the RC: bottom side, pins down
    "J2": (80.0, 90.0, 90, "bottom"),
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
    # charge up to 2.1 A, boost input ~1.5 A at 0.5 A out: 1 mm on 1 oz
    ("Battery", 1.0, 0.2, 0.8, 0.4, ["BATT_P", "BATT_N", "SW", "DW_DRAIN", "+5V", "GND"]),
    # USB VBUS: the HRO connector's VBUS pads are 0.6 mm wide
    ("Vbus", 0.5, 0.2, 0.8, 0.4, ["VIN"]),
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


def add_rect(board, x0, y0, x1, y1, layer=pcbnew.F_SilkS):
    for a, b in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
        add_line(board, a, b, layer)


def add_silkscreen(board):
    """Polarity, the controls and what this board plugs into."""
    for fp in board.Footprints():
        if fp.GetReference().startswith(("H", "J", "D", "SW")):
            fp.Reference().SetVisible(False)
    j1 = board.FindFootprintByReference("J1")      # USB-C overhangs the east edge
    for item in list(j1.GraphicalItems()):
        if item.GetLayer() == pcbnew.F_SilkS and pcbnew.ToMM(item.GetBoundingBox().GetRight()) > X0 + W - 0.3:
            j1.Remove(item)
    # a reversed cell is not protected: say it big, at both ends
    add_text(board, "+", 146.0, 63.0, size=3.0)
    add_text(board, "-", 56.0, 63.0, size=3.0)
    add_text(board, "18650   + this end", 115.0, 63.0, size=1.2)
    add_text(board, "25  50  75 100 %", 98.0, 91.6, size=0.8)
    add_text(board, "POWER: press = on, hold 2 s = off", 98.0, 76.0, size=0.8)
    add_text(board, "FalconDrone RC power rev A", 70.0, 92.0, size=0.8)
    add_text(board, "to RC: 5V G VBAT KEEP", 80.0, 87.2, bottom=True, size=0.8)



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
        if ref == "U1":
            # the exposed pad's thermal vias are 0.2 mm; JLCPCB's standard minimum is 0.3
            for pad in fp.Pads():
                if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and pad.GetDrillSize().x < mm(0.3):
                    pad.SetDrillSize(pcbnew.VECTOR2I(mm(0.3), mm(0.3)))
                    pad.SetSize(pcbnew.F_Cu, pcbnew.VECTOR2I(mm(0.6), mm(0.6)))
        if ref in PLACEMENT:
            place(fp, *PLACEMENT[ref])
        elif ref.startswith("H") and ref[1:].isdigit():
            place(fp, *HOLES[int(ref[1:]) - 1], 0, "top")
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
