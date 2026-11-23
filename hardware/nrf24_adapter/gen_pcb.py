#!/usr/bin/env python3
"""
Create nrf24_adapter.kicad_pcb: 26 x 16 mm, 2 layers, GND pours.

The module plugs into a 2x4 female socket at the west end and lies over the
board, its antenna past the east edge. The 1x8 straight header runs along the
south edge (pins down): GND VCC CE CSN SCK MOSI MISO IRQ, west to east.
C1/C2 sit next to the VCC pin, under the module.

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

PROJECT = "nrf24_adapter"
FP_DIR = "/usr/share/kicad/footprints"
X0, Y0, W, H = 50.0, 50.0, 26.0, 16.0
CORNER_R = 1.5
HOLE_INSET = 4.0               # corner hole centre from each edge

HDR_X = X0 + W / 2 - 3.5 * 2.54   # 1x8 pin 1, header centred

mm = pcbnew.FromMM

# ref: (x, y, rotation_deg, side). Coordinates in mm, KiCad frame (y down),
# as seen from the top. Rotation is counter-clockwise as shown in KiCad;
# a 1xN header at 90 runs east from pin 1.
PLACEMENT = {
    "J1": (52.5, 52.0, 0, "top"),            # module socket, pin 1 (GND) north-west
    "J2": (HDR_X, 63.5, 90, "bottom"),       # 1x8 out, pins down, runs east
    "C1": (60.0, 54.0, 90, "top"),
    "C2": (63.0, 54.0, 90, "top"),
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
    ("Power", 0.5, 0.2, 0.8, 0.4, ["+3V3", "GND"]),
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
    """Pin names along the 1x8, and drop the module outline that hangs off the board."""
    for fp in board.Footprints():
        fp.Reference().SetVisible(False)          # four parts: the labels say it all
    # the module outline would cross the 1x8 labels: draw just the socket
    j1 = board.FindFootprintByReference("J1")
    for item in list(j1.GraphicalItems()):
        if item.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
            j1.Remove(item)
    x, y = PLACEMENT["J1"][:2]
    for a, b in (((x - 1.5, y - 1.5), (x + 4.04, y - 1.5)), ((x + 4.04, y - 1.5), (x + 4.04, y + 9.12)),
                 ((x + 4.04, y + 9.12), (x - 1.5, y + 9.12)), ((x - 1.5, y + 9.12), (x - 1.5, y - 1.5))):
        add_line(board, a, b)
    for k, name in enumerate(("G", "V", "CE", "CS", "SK", "MO", "MI", "IQ")):
        add_text(board, name, HDR_X + 2.54 * k, 61.4, size=0.8)
    add_text(board, "nRF24", 68.5, 53.0, size=0.9)
    add_text(board, "3V3 only", 68.5, 55.2, size=0.8)
    # seen from the bottom the row is mirrored: pin 1 (GND) is the east end
    add_text(board, "IRQ MISO MOSI SCK CSN CE VCC GND", HDR_X + 3.5 * 2.54, 61.0, bottom=True, size=0.8)
    add_text(board, "FalconDrone", 66.0, 53.5, bottom=True, size=0.8)
    add_text(board, "nRF24 adapter", 66.0, 55.5, bottom=True, size=0.8)


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
            fp.SetExcludedFromBOM(False)   # the wire-pad footprint says so; it is a real part here
        if ref == "J1":
            # the module rides 8.5 mm up on its socket, over C1/C2 and the 1x8
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
