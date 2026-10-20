#!/usr/bin/env python3
"""
Create falcon_fc.kicad_pcb: board outline, 4-layer stackup, JLCPCB rules,
mounting holes, ground/power planes, part placement and silkscreen.

The board is built in three steps, all regenerated from scripts:
  gen_pcb.py --force    placed, unrouted board (this script)
  route_pcb.py          fanout + Freerouting + maze finisher -> routed board
  export_fab.py         DRC check, Gerbers/drill zip, BOM and CPL for JLCPCB
Rerunning this script overwrites the board (it refuses to if the board already
has tracks, unless given --force). Hand edits made in KiCad are lost then.

Board: 45 x 45 mm, 30.5 mm mounting pattern (4 mm holes for M3 grommets), centred on (100, 100).
  Top    : GPS in the middle (sky view, solid GND under it), power entry
           north, ESC header east, AUX JST-GH west, nRF24 socket south with
           the module overhanging the edge.
  Bottom : MCU under the GPS (shielded by the In1 GND plane), IMU next to it
           near the centre of mass, baro away from the regulators.
Layers : F.Cu signals | In1.Cu GND plane | In2.Cu +3V3 plane | B.Cu signals.

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

PROJECT = "falcon_fc"
FP_DIR = "/usr/share/kicad/footprints"
CX, CY = 100.0, 100.0          # board centre
SIZE = 45.0                    # board edge length
CORNER_R = 2.0
HOLE_PITCH = 30.5

mm = pcbnew.FromMM

# ref: (x, y, rotation_deg, side). Coordinates in mm, KiCad frame (y down),
# as seen from the top. Rotation is counter-clockwise as shown in KiCad.
PLACEMENT = {
    # ---------------- top ----------------
    "U7": (100.0, 100.0, 0, "top"),        # GPS, centred
    # power entry, north edge
    "J1": (100.0, 80.8, 180, "top"),       # JST-GH, opening faces north
    "Q1": (91.5, 81.6, 0, "top"),
    "D2": (108.0, 80.6, 0, "top"),
    "U2": (100.0, 88.0, 0, "top"),
    "C1": (92.5, 87.0, 90, "top"),
    "C2": (107.5, 87.0, 90, "top"),
    # GPS LDO, north-east, next to GPS VCC (pin 17)
    "U6": (113.5, 91.2, 0, "top"),
    "C23": (113.5, 93.8, 0, "top"),
    "C24": (110.6, 89.6, 90, "top"),
    "C25": (110.0, 93.6, 90, "top"),
    "C26": (106.5, 90.8, 0, "top"),
    # connectors
    "J4": (120.5, 94.92, 0, "top"),        # ESC, east edge, pins run south
    # AUX JST-GH, opening faces west; low on the edge so its nets head for the
    # MCU's south pins (PA8-PA10, PC9) without crossing the IMU cluster below.
    "J3": (81.0, 104.0, 270, "top"),
    "U9": (88.4, 107.0, 270, "top"),       # its ESD array, pins in J3 order
    "R9": (89.9, 111.0, 90, "top"),        # I2C3 pull-ups
    "R10": (91.6, 110.4, 90, "top"),
    "J2": (108.6, 114.0, 270, "top"),      # nRF24 socket, module hangs south
    # user interface, south-west
    "SW2": (91.8, 119.2, 0, "top"),        # CAL button
    "D1": (92.5, 114.2, 0, "top"),         # status LED

    # ---------------- bottom ----------------
    "U1": (100.0, 100.0, 270, "bottom"),   # MCU under the GPS. 270 deg puts SPI3
                                           # west (IMU), SPI2 south (nRF24),
                                           # motor pins north/east (ESC header)
    # IMU group, west of the MCU next to SPI3 (pins 50-53)
    "U3": (88.2, 102.5, 0, "bottom"),
    "C14": (88.2, 99.2, 0, "bottom"),
    "C15": (88.2, 105.8, 0, "bottom"),
    "C16": (85.0, 101.0, 90, "bottom"),
    "U4": (87.0, 91.5, 0, "bottom"),       # sensor LDO
    "C20": (91.0, 89.0, 0, "bottom"),
    "C21": (83.2, 94.5, 90, "bottom"),
    # baro, south-west, far from all three regulators
    "U5": (91.0, 117.0, 0, "bottom"),
    "C22": (94.6, 115.0, 0, "bottom"),
    "R7": (94.6, 117.2, 0, "bottom"),
    "R8": (94.6, 119.4, 0, "bottom"),
    # debug connectors, north edge (power J1 is on the top side there)
    "J5": (95.5, 80.8, 0, "bottom"),     # SWD
    "J6": (106.5, 80.8, 0, "bottom"),    # telemetry
    "U10": (106.5, 86.5, 90, "bottom"),    # its ESD array
    # east: ESC ESD, battery divider, reset
    "U8": (116.0, 100.0, 90, "bottom"),
    "R1": (112.0, 94.4, 0, "bottom"),
    "R2": (112.0, 96.6, 0, "bottom"),
    "C3": (112.0, 98.8, 0, "bottom"),
    "SW1": (114.5, 108.5, 0, "bottom"),
    # nRF24 supply caps, at its pins
    "C18": (106.0, 119.2, 0, "bottom"),
    "C19": (101.6, 119.2, 0, "bottom"),
    # LED resistor, CAL button pull-up + filter (SW2/D1 are above, on top)
    # MCU VDD decoupling across the package corners, where there are no pins:
    # straight out from a pin they wall off the neighbouring signals' escape.
    # Each sits by the corner VDD pin (64 NW, 19 NE, 32 SE, 48 SW) and reaches
    # it through its own via into the In2 plane.
    "C7": (93.1, 93.1, 45, "bottom"),
    "C8": (91.0, 92.2, 45, "bottom"),      # VDD bulk
    "C4": (106.9, 93.1, 135, "bottom"),
    "C5": (106.9, 106.9, 45, "bottom"),
    "C6": (93.1, 106.9, 135, "bottom"),
    "R4": (97.8, 112.0, 0, "bottom"),
    "R5": (97.8, 114.2, 0, "bottom"),
    "C13": (97.8, 116.4, 0, "bottom"),
}

# MCU decoupling goes next to the pad it serves:
# ref: (target ref, pad, distance out from the pad, shift along the package edge).
# Two-terminal parts are turned so their long axis points away from the package.
NEAR_PAD = {
    "R3": ("U1", "60", 2.8, 0.0),      # BOOT0
    "C10": ("U1", "13", 2.8, 0.0),     # VDDA
    "C11": ("U1", "13", 2.8, 1.6),     # VDDA bulk
    "FB1": ("U1", "13", 6.0, 0.0),     # VDDA filter
    "C12": ("U1", "7", 2.8, 0.0),      # NRST
    "C9": ("U1", "30", 2.8, 0.0),      # VCAP_1
}

IN2_WINDOW = (85.8, 99.0, 93.6, 105.5)     # the only place In2 may carry tracks

MOUNT_FP = ("MountingHole", "MountingHole_4mm")    # M3 + rubber grommets

# JLCPCB 4-layer standard capabilities (no extra-cost options), with margin.
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
    ("Power", 0.5, 0.15, 0.8, 0.4, ["+5V", "+5V_IN", "+3V3", "VBAT", "GND"]),
    ("Sensor_power", 0.25, 0.15, 0.6, 0.3, ["+3V3_SENS", "+3V3_GPS", "+3V3A", "VCAP1"]),
]


def load_netlist():
    path = os.path.join(HERE, "build", PROJECT + ".net")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    subprocess.run(["kicad-cli", "sch", "export", "netlist", "-o", path,
                    os.path.join(HERE, PROJECT + ".kicad_sch")], check=True, capture_output=True)
    t = sch.parse_sexp(open(path).read())
    comps = {}
    for c in sch.find(sch.find1(t, "components"), "comp"):
        comps[str(sch.find1(c, "ref")[1])] = (str(sch.find1(c, "value")[1]),
                                              str(sch.find1(c, "footprint")[1]))
    nets = {}
    for n in sch.find(sch.find1(t, "nets"), "net"):
        name = str(sch.find1(n, "name")[1])           # local labels come as "/NAME"
        nets[name] = [(str(sch.find1(x, "ref")[1]), str(sch.find1(x, "pin")[1]))
                      for x in sch.find(n, "node")]
    return comps, nets


def add_outline(board):
    h = SIZE / 2
    x0, y0, x1, y1 = CX - h, CY - h, CX + h, CY + h
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
    h = SIZE / 2 - 0.3
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net)
    z.SetAssignedPriority(priority)
    z.SetMinThickness(mm(0.2))
    z.SetLocalClearance(mm(0.2))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.25))
    z.SetThermalReliefSpokeWidth(mm(0.3))
    ol = z.Outline()
    ol.NewOutline()
    for x, y in ((CX - h, CY - h), (CX + h, CY - h), (CX + h, CY + h), (CX - h, CY + h)):
        ol.Append(mm(x), mm(y))
    board.Add(z)
    return z


def add_track_keepout(board, layer, x0, y0, x1, y1):
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True)
    z.SetLayer(layer)
    z.SetDoNotAllowTracks(True)
    z.SetDoNotAllowVias(False)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowCopperPour(False)
    z.SetDoNotAllowFootprints(False)
    ol = z.Outline()
    ol.NewOutline()
    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        ol.Append(mm(x), mm(y))
    board.Add(z)


def add_text(board, text, x, y, bottom=False, angle=0):
    txt = pcbnew.PCB_TEXT(board)
    txt.SetText(text)
    txt.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    txt.SetLayer(pcbnew.B_SilkS if bottom else pcbnew.F_SilkS)
    txt.SetMirrored(bottom)
    txt.SetTextAngleDegrees(angle)
    txt.SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
    txt.SetTextThickness(mm(0.15))
    board.Add(txt)


def add_silk_line(board, a, b):
    s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
    s.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
    s.SetLayer(pcbnew.F_SilkS)
    s.SetWidth(mm(0.2))
    board.Add(s)


def add_silkscreen(board):
    """Labels for the person holding the board; JLCPCB places parts from the CPL,
    so passives (too small to label legibly here) lose their reference text."""
    for fp in board.Footprints():
        ref = fp.GetReference()
        # also refs with no room clear of other silk; the parts are obvious anyway
        if ref.startswith(("R", "C", "FB")) or ref in ("J1", "J6", "U2", "U7", "U10", "D2"):
            fp.Reference().SetVisible(False)
    # The nRF24 socket's footprint outlines the whole module, which overhangs the
    # south edge; drop the silk that would land off the board.
    edge = CY + SIZE / 2
    j2 = board.FindFootprintByReference("J2")
    for item in list(j2.GraphicalItems()):
        if item.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS) and \
                pcbnew.ToMM(item.GetBoundingBox().GetBottom()) > edge - 0.3:
            j2.Remove(item)
    # Forward is north: the power lead and the SWD/telemetry plugs face forward.
    add_silk_line(board, (82.5, 96.3), (82.5, 90.5))
    add_silk_line(board, (82.5, 90.5), (81.7, 91.5))
    add_silk_line(board, (82.5, 90.5), (83.3, 91.5))
    add_text(board, "FWD", 82.5, 97.2)
    add_text(board, "FalconDrone FC rev A", CX, CY + 10.2)
    j4 = board.FindFootprintByReference("J4")
    for pad, name in zip(sorted(j4.Pads(), key=lambda p: p.GetPosition().y), ("M1", "M2", "M3", "M4", "G")):
        add_text(board, name, 117.4, pcbnew.ToMM(pad.GetPosition().y))
    add_text(board, "AUX", 80.6, 110.9)
    add_text(board, "SWD", 88.4, 80.4, bottom=True)
    add_text(board, "TLM", 111.6, 80.4, bottom=True)


def place(fp, x, y, rot, side):
    if side == "bottom":
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    fp.SetOrientationDegrees(rot)


def place_near_pad(board, fp, target_ref, pad_num, dist, shift):
    target = board.FindFootprintByReference(target_ref)
    pad = next(p for p in target.Pads() if p.GetNumber() == pad_num)
    px, py = pcbnew.ToMM(pad.GetPosition().x), pcbnew.ToMM(pad.GetPosition().y)
    tx, ty = pcbnew.ToMM(target.GetPosition().x), pcbnew.ToMM(target.GetPosition().y)
    # Push straight out from whichever side of the package the pad is on.
    if abs(px - tx) > abs(py - ty):
        x, y, rot = px + math.copysign(dist, px - tx), py + shift, 0
    else:
        x, y, rot = px + shift, py + math.copysign(dist, py - ty), 90
    place(fp, x, y, rot, "bottom" if target.IsFlipped() else "top")
    # Turn it round if the pad on the pin's net ended up on the far side.
    near = lambda q: math.hypot(pcbnew.ToMM(q.GetPosition().x) - px, pcbnew.ToMM(q.GetPosition().y) - py)
    mine = [q for q in fp.Pads() if q.GetNetname() == pad.GetNetname()]
    if mine and near(mine[0]) > min(near(q) for q in fp.Pads()):
        fp.SetOrientationDegrees(rot + 180)


def patch_project_rules():
    path = os.path.join(HERE, PROJECT + ".kicad_pro")
    pro = json.load(open(path))
    ds = pro.setdefault("board", {}).setdefault("design_settings", {})
    ds.setdefault("rules", {}).update(RULES)
    ns = pro.setdefault("net_settings", {})
    ns["classes"] = [{
        "name": name, "track_width": tw, "clearance": cl, "via_diameter": vd, "via_drill": vdr,
        "microvia_diameter": 0.3, "microvia_drill": 0.1, "diff_pair_width": 0.2,
        "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25, "wire_width": 6, "bus_width": 12,
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
    board.SetCopperLayerCount(4)
    board.SetLayerName(pcbnew.In1_Cu, "GND")
    board.SetLayerName(pcbnew.In2_Cu, "3V3")
    board.GetDesignSettings().SetBoardThickness(mm(1.6))

    netinfo = {}
    for name in nets:
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        netinfo[name] = ni
    pad_net = {(r, p): name for name, nodes in nets.items() for r, p in nodes}

    add_outline(board)

    unplaced = []
    for ref, (value, fpid) in sorted(comps.items()):
        lib, name = fpid.split(":")
        fp = pcbnew.FootprintLoad(f"{FP_DIR}/{lib}.pretty", name)
        fp.SetReference(ref)
        fp.SetValue(value)
        fp.SetFPID(pcbnew.LIB_ID(lib, name))
        board.Add(fp)
        for pad in fp.Pads():
            net = pad_net.get((ref, pad.GetNumber()))
            if net:
                pad.SetNet(netinfo[net])
        if ref in PLACEMENT:
            place(fp, *PLACEMENT[ref])
        elif ref not in NEAR_PAD:
            unplaced.append(ref)
    for ref, (tref, pad, dist, shift) in NEAR_PAD.items():
        place_near_pad(board, board.FindFootprintByReference(ref), tref, pad, dist, shift)

    # Mounting holes: 4 mm NPTH for M3 soft-mount grommets (vibration isolation
    # for the IMU), not tied to GND (no ground loop through the frame).
    lib, name = MOUNT_FP
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        fp = pcbnew.FootprintLoad(f"{FP_DIR}/{lib}.pretty", name)
        fp.SetReference(f"H{i + 1}")
        fp.SetBoardOnly(True)              # not in the schematic
        fp.SetPosition(pcbnew.VECTOR2I(mm(CX + sx * HOLE_PITCH / 2), mm(CY + sy * HOLE_PITCH / 2)))
        board.Add(fp)

    # Planes: In1 solid GND, In2 +3V3, GND pours on both outer layers.
    add_zone(board, pcbnew.In1_Cu, netinfo["/GND"])
    add_zone(board, pcbnew.In2_Cu, netinfo["/+3V3"])
    add_zone(board, pcbnew.F_Cu, netinfo["/GND"])
    add_zone(board, pcbnew.B_Cu, netinfo["/GND"])
    # u-blox: keep the host copper under the GPS module solid GND (no top-side
    # tracks inside its pad ring).
    add_track_keepout(board, pcbnew.F_Cu, CX - 5.4, CY - 5.4, CX + 5.4, CY + 5.4)
    # In2 (+3V3) stays a plane except one window between the IMU and the MCU's
    # SPI3 pins: the two pin rings order the bus differently, so some lines have
    # to cross, and this gives them a layer to do it on without leaving the
    # corridor (In1 GND stays solid underneath as their reference).
    wx0, wy0, wx1, wy1 = IN2_WINDOW
    e0, e1 = CX - SIZE / 2, CX + SIZE / 2
    for x0, y0, x1, y1 in ((e0, e0, e1, wy0), (e0, wy1, e1, e1), (e0, wy0, wx0, wy1), (wx1, wy0, e1, wy1)):
        add_track_keepout(board, pcbnew.In2_Cu, x0, y0, x1, y1)

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
